using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SimulationApiSchema;
using System.Threading;

namespace AWSIM
{
    /// <summary>
    /// Class to process client (Scenario Simulator v2) requests.
    /// </summary>
    public class ScenarioSimulatorRequestProcessor : MonoBehaviour
    {
        #region [Helper Class]

        /// <summary>
        /// Entity's assetKey and prefab pair class.
        /// Dictionaries are not available in Unity's inspector, so use classes instead.
        /// </summary>
        [Serializable]
        private class EntityPrefab
        {
            public string AssetKey;
            public GameObject Prefab;
        }

        #endregion

        #region [Inspector Vars]

        [Header("Entities")]
        [SerializeField] private EntityPrefab[] entityPrefabs;
        [SerializeField] private Transform entitesRoot;
        [SerializeField] private FollowCamera egoFollowCamera;

        [Space(10)]
        [Header("Execution Type")]
        [SerializeField] private bool stepExecution = false;

        /// <summary>
        /// How much UpdateFrame step is covered by TimeScale > 0, the rest part has TimeScale = 0.
        /// </summary>
        [Range(0f,1f)]
        [SerializeField] private float stepDurationInPercentage = 0.5f;

        #endregion

        #region [Variables]

        // Prefab is in Unity Editor.
        private Dictionary<string, GameObject> entityPrefabDic = new Dictionary<string, GameObject>();      // <unique_id, entity asset>
        
        // Instance is game object in Scene.
        private Dictionary<string, GameObject> entityInstanceDic = new Dictionary<string, GameObject>();    // <unique_id, entity instance>

        // main thread context
        private SynchronizationContext mainContext = default;

        private static readonly object lockObject = new object();
        private static readonly object lockOnEntityDic = new object();
        private static readonly object lockOnFrameUpdate = new object();

        // time source
        private ExternalTimeSource timeSource = default;
  
        #endregion

        #region [Traffic lights]

        private Dictionary<long, TrafficLight> trafficLights;
        private List<long> modifiedTrafficLights;
        private void preprocessTrafficLights() {
            modifiedTrafficLights = new List<long>();
            trafficLights = new Dictionary<long, TrafficLight>();
            var trafficLightObjects = FindObjectsOfType<TrafficLightLaneletID>();
            for (int i = 0; i < trafficLightObjects.Length; i++) {
                AWSIM.TrafficLightLaneletID laneletId = trafficLightObjects[i];
                GameObject obj = laneletId.gameObject;
                TrafficLight tl = obj.GetComponent<TrafficLight>();
                if (tl != null && laneletId != null) {
                    trafficLights.Add(laneletId.wayID, tl);
                }
            }
        }

        private void resetSingleTraffcLight(TrafficLight light) {
            light.SetBulbData(
                new TrafficLight.BulbData(TrafficLight.BulbType.GREEN_BULB,  TrafficLight.BulbColor.GREEN, TrafficLight.BulbStatus.SOLID_OFF));
            light.SetBulbData(
                new TrafficLight.BulbData(TrafficLight.BulbType.RED_BULB, TrafficLight.BulbColor.RED, TrafficLight.BulbStatus.SOLID_OFF));
            light.SetBulbData(
                new TrafficLight.BulbData(TrafficLight.BulbType.YELLOW_BULB, TrafficLight.BulbColor.YELLOW, TrafficLight.BulbStatus.SOLID_OFF));
        }

        private void resetModifiedTrafficLights() {
            foreach(var trafficLightId in modifiedTrafficLights) {
                resetSingleTraffcLight(trafficLights[trafficLightId]);
            }
            modifiedTrafficLights.Clear();
        }

        private void resetAllTrafficLights() {
            foreach(var light in trafficLights) {
                resetSingleTraffcLight(light.Value);
            }
        }

        private TrafficLight.BulbData fromProto(SimulationApiSchema.TrafficLight proto) {
            var t = new TrafficLight.BulbType();
            switch (proto.Shape) {
                case SimulationApiSchema.TrafficLight.Types.Shape.Circle:            t = TrafficLight.BulbType.ANY_CIRCLE_BULB;        break;
                case SimulationApiSchema.TrafficLight.Types.Shape.LeftArrow:         t = TrafficLight.BulbType.LEFT_ARROW_BULB;        break;
                case SimulationApiSchema.TrafficLight.Types.Shape.RightArrow:        t = TrafficLight.BulbType.RIGHT_ARROW_BULB;       break;
                case SimulationApiSchema.TrafficLight.Types.Shape.UpArrow:           t = TrafficLight.BulbType.UP_ARROW_BULB;          break;
                case SimulationApiSchema.TrafficLight.Types.Shape.DownArrow:         t = TrafficLight.BulbType.DOWN_ARROW_BULB;        break;
                case SimulationApiSchema.TrafficLight.Types.Shape.DownLeftArrow:     t = TrafficLight.BulbType.DOWN_LEFT_ARROW_BULB;   break;
                case SimulationApiSchema.TrafficLight.Types.Shape.DownRightArrow:    t = TrafficLight.BulbType.DOWN_RIGHT_ARROW_BULB;  break;
                case SimulationApiSchema.TrafficLight.Types.Shape.Cross:             t = TrafficLight.BulbType.CROSS_BULB;             break;
            }

            var c = new TrafficLight.BulbColor();
            switch (proto.Color) {
                case SimulationApiSchema.TrafficLight.Types.Color.Red:   c = TrafficLight.BulbColor.RED;    t = TrafficLight.BulbType.RED_BULB;    break;
                case SimulationApiSchema.TrafficLight.Types.Color.Amber: c = TrafficLight.BulbColor.YELLOW; t = TrafficLight.BulbType.YELLOW_BULB; break;
                case SimulationApiSchema.TrafficLight.Types.Color.Green: c = TrafficLight.BulbColor.GREEN;  t = TrafficLight.BulbType.GREEN_BULB;  break;
                case SimulationApiSchema.TrafficLight.Types.Color.White: c = TrafficLight.BulbColor.WHITE;                                         break;
            } 

            var s = new TrafficLight.BulbStatus();
            switch (proto.Status) {
                case SimulationApiSchema.TrafficLight.Types.Status.SolidOff: s = TrafficLight.BulbStatus.SOLID_OFF; break;
                case SimulationApiSchema.TrafficLight.Types.Status.SolidOn:  s = TrafficLight.BulbStatus.SOLID_ON;  break;
                case SimulationApiSchema.TrafficLight.Types.Status.Flashing: s = TrafficLight.BulbStatus.FLASHING;  break;
            }
            return new TrafficLight.BulbData(t, c, s);
        }

        #endregion

        #region [Mutables]

        private bool isInitialized = false;  // has the initialize request been received

        // time
        private float realtimeFactor = 1f;
        private double fixedDeltaTime;
        private double prevUpdateFrameTime = 0d;


        // step execution - fixedUpdate frame counter

        private int fixedUpdateCount = 0;
        private int targetFixedUpdateCount = 0;
        private bool isFixedUpdating = false;

        #endregion


        // ------      METHODS      ------ //

        #region [Life Cycle]

        public void Initialize()
        {
             // get time source from time source provide
            timeSource = TimeSourceProvider.GetTimeSource() as ExternalTimeSource;
            if(timeSource == null)
            {
                Debug.LogError("Scenario Simulator requires time source of type SS2. Check if TimeSourceSelector is on the scene and SS2 is the selected as the Time Source");
            }
            timeSource.Initialize();
            
            mainContext = SynchronizationContext.Current;

            fixedDeltaTime = Time.fixedDeltaTime;

            Time.timeScale = stepExecution? 0f : 1f;

            entityPrefabDic = new Dictionary<string, GameObject>(); 
            entityPrefabDic = entityPrefabs.ToDictionary(x => x.AssetKey, y => y.Prefab);

            entityInstanceDic = new Dictionary<string, GameObject>();
            preprocessTrafficLights();
            resetAllTrafficLights();
        }

        public void Dispose()
        {
            if(entityPrefabDic != null)
            {
                entityPrefabDic.Clear();
                entityPrefabDic = null;
            }

            if(entityInstanceDic != null)
            {
                entityInstanceDic.Clear();
                entityInstanceDic = null;
            }

            Time.timeScale = 1f;

            mainContext = null;
            timeSource.Dispose();
        }

        #endregion

        #region [Public Methods]

        public SimulationResponse Process(SimulationRequest request)
        {
            SimulationResponse response = new SimulationResponse();

            switch(request.RequestCase) 
            {
                case SimulationRequest.RequestOneofCase.Initialize:
                    response.Initialize = Initialize(request.Initialize);
                    break;
                case SimulationRequest.RequestOneofCase.UpdateFrame:
                    response.UpdateFrame = UpdateFrame(request.UpdateFrame);
                    break;
                case SimulationRequest.RequestOneofCase.SpawnVehicleEntity:
                    response.SpawnVehicleEntity = SpawnVehicleEntity(request.SpawnVehicleEntity);
                    break;
                case SimulationRequest.RequestOneofCase.SpawnPedestrianEntity:
                    response.SpawnPedestrianEntity = SpawnPedestrianEntity(request.SpawnPedestrianEntity);
                    break;
                case SimulationRequest.RequestOneofCase.SpawnMiscObjectEntity:
                    response.SpawnMiscObjectEntity = SpawnMiscObjectEntity(request.SpawnMiscObjectEntity);
                    break;
                case SimulationRequest.RequestOneofCase.UpdateEntityStatus:
                    response.UpdateEntityStatus = UpdateEntityStatus(request.UpdateEntityStatus);
                    break;
                case SimulationRequest.RequestOneofCase.AttachLidarSensor:
                    response.AttachLidarSensor = AttachLidarSensor(request.AttachLidarSensor);
                    break;
                case SimulationRequest.RequestOneofCase.AttachDetectionSensor:
                    response.AttachDetectionSensor = AttachDetectionSensor(request.AttachDetectionSensor);
                    break;
                case SimulationRequest.RequestOneofCase.AttachOccupancyGridSensor:
                    response.AttachOccupancyGridSensor = AttachOccupancyGridSensor(request.AttachOccupancyGridSensor);
                    break;
                case SimulationRequest.RequestOneofCase.DespawnEntity:
                    response.DespawnEntity = DespawnEntity(request.DespawnEntity);
                    break;
                case SimulationRequest.RequestOneofCase.UpdateTrafficLights:
                    response.UpdateTrafficLights = UpdateTrafficLights(request.UpdateTrafficLights);
                    break;
                case SimulationRequest.RequestOneofCase.AttachPseudoTrafficLightDetector:
                    response.AttachPseudoTrafficLightDetector = UpdateTrafficLights(request.AttachPseudoTrafficLightDetector);
                    break;
                default:
                    Debug.Log("Case " + request.RequestCase.ToString() + " not yet supported");
                    break;
            }

            return response;

        }

        #endregion

        #region [Private Methods]

        private InitializeResponse Initialize(InitializeRequest request) 
        {
            realtimeFactor = (float)request.RealtimeFactor;
            if(stepExecution)
            {
                realtimeFactor *= (1f / stepDurationInPercentage);
            }

            // set previous time and soruce time to current time from request
            prevUpdateFrameTime = request.InitializeTime;
            BuiltinInterfaces.Time currentRosTime = request.InitializeRosTime;
            timeSource.SetTime(currentRosTime.Sec, currentRosTime.Nanosec);

            mainContext.Send(_ =>
            {
                lock (lockOnFrameUpdate)
                {
                    Time.timeScale = stepExecution? 0f : realtimeFactor;
                }
            }, null);

            // return response.
            var initializeResponse = new InitializeResponse()
            {
                Result = new Result()
                {
                    Success = true,
                    Description = "AWSIM initialized",
                }
            };
            
            isInitialized = true;
            return initializeResponse;
        }

        private UpdateFrameResponse UpdateFrame(UpdateFrameRequest request)
        {            
            BuiltinInterfaces.Time currentRosTime = request.CurrentRosTime;
            timeSource.SetTime(currentRosTime.Sec, currentRosTime.Nanosec);   

            // calculate how many frames to update
            if (stepExecution)
            {
                double elapsedSec = Math.Abs(Math.Abs(request.CurrentSimulationTime) - Math.Abs(prevUpdateFrameTime));
                prevUpdateFrameTime = request.CurrentSimulationTime;

                // start time flow
                mainContext.Send(_ =>
                {
                    lock (lockOnFrameUpdate)
                    {
                        //isFixedUpdating = true;
                        Time.timeScale = realtimeFactor;
                    }
                }, null);

                int waitTime = Mathf.CeilToInt((float) (elapsedSec * 1000.0 * stepDurationInPercentage));
                Thread.Sleep(waitTime);

                // freez time flow
                mainContext.Send(_ =>
                {
                    lock(lockOnFrameUpdate)
                    {
                        Time.timeScale = 0f;
                    }
                }, null);
            }
      
            var frameUpdateResponse = new UpdateFrameResponse()
            {
                Result = new Result()
                {
                    Success = true,
                    Description = "",
                }
            };
            return frameUpdateResponse;
        }

        private UpdateEntityStatusResponse UpdateEntityStatus(UpdateEntityStatusRequest request) 
        {
            var updateEntityStatusResponse = new UpdateEntityStatusResponse();

            mainContext.Send(_ =>
            {
                for (int i = 0; i < request.Status.Count; i++) 
                {
                    var reqStatus = request.Status[i];

                    var status = new UpdatedEntityStatus();

                    var unityPose = GetUnityPose(reqStatus.Pose);
                    var instance = entityInstanceDic[reqStatus.Name];

                    if (instance.TryGetComponent(out NPCVehicle vehicle))
                    {
                        vehicle.SetPosition(unityPose.position);
                        vehicle.SetRotation(unityPose.rotation);
                        status.Name = reqStatus.Name;
                        status.Pose = reqStatus.Pose;
                        status.ActionStatus = reqStatus.ActionStatus;
                    }
                    else if (instance.TryGetComponent(out NPCPedestrian pedestrian))
                    {
                        pedestrian.SetPosition(unityPose.position);
                        pedestrian.SetRotation(unityPose.rotation);
                        status.Name = reqStatus.Name;
                        status.Pose = reqStatus.Pose;
                        status.ActionStatus = reqStatus.ActionStatus;
                    }
                    else if (instance.TryGetComponent(out Vehicle ego_vehicle))
                    {
                        status.Name = reqStatus.Name;
                        status.Pose = GetRosPose(instance.transform);
                        status.ActionStatus = GetEgoActionStatus(ego_vehicle, reqStatus.ActionStatus);
                    }
                    else 
                    {
                        instance.transform.position = unityPose.position;
                        instance.transform.rotation = unityPose.rotation;
                        status.Name = reqStatus.Name;
                        status.Pose = reqStatus.Pose;
                        status.ActionStatus = reqStatus.ActionStatus;
                    }

                    lock(lockObject) 
                    {
                        updateEntityStatusResponse.Status.Add(status);
                    }
                }
                
            }, null);

            updateEntityStatusResponse.Result = new Result()
            {
                Success = true,
                Description = "",
  
            };

            return updateEntityStatusResponse;
        }

        private SpawnVehicleEntityResponse SpawnVehicleEntity(SpawnVehicleEntityRequest request) 
        {
            var asset_key = request.AssetKey;
            var pose = request.Pose;

            mainContext.Send(_ =>
            {
                lock (lockOnEntityDic)
                {
                    var unityPose = GetUnityPose(pose);
                    var prefab = entityPrefabDic[asset_key];
                    var instance = GameObject.Instantiate(prefab, unityPose.position, unityPose.rotation, entitesRoot);
                    entityInstanceDic.Add(request.Parameters.Name, instance);

                    if (request.IsEgo)
                        egoFollowCamera.target = instance.transform;
                }
            }, null);

            var spawnVehicleResponse = new SpawnVehicleEntityResponse()
            {
                Result = new Result()
                {
                    Success = true,
                    Description = "",
                }
            };
            return spawnVehicleResponse;
        }

        private SpawnPedestrianEntityResponse SpawnPedestrianEntity(SpawnPedestrianEntityRequest request) 
        {
            var asset_key = request.AssetKey;
            var pose = request.Pose;

            mainContext.Send(_ =>
            {
                lock (lockOnEntityDic)
                {
                    var unityPose = GetUnityPose(pose);
                    var prefab = entityPrefabDic[asset_key];
                    var instance = GameObject.Instantiate(prefab, unityPose.position, unityPose.rotation, entitesRoot);
                    entityInstanceDic.Add(request.Parameters.Name, instance);
                }
            }, null);

            var spawnPedestrianResponse = new SpawnPedestrianEntityResponse()
            {
                Result = new Result()
                {
                    Success = true,
                    Description = "",
                }
            };
            return spawnPedestrianResponse;
        }

        private SpawnMiscObjectEntityResponse SpawnMiscObjectEntity(SpawnMiscObjectEntityRequest request) 
        {
            var asset_key = request.AssetKey;
            var pose = request.Pose;

            mainContext.Send(_ =>
            {
                lock (lockOnEntityDic)
                {
                    var unityPose = GetUnityPose(pose);
                    var prefab = entityPrefabDic[asset_key];
                    var instance = GameObject.Instantiate(prefab, unityPose.position, unityPose.rotation, entitesRoot);
                    entityInstanceDic.Add(request.Parameters.Name, instance);
                }
            }, null);

            var spawnMiscObjectResponse = new SpawnMiscObjectEntityResponse()
            {
                Result = new Result()
                {
                    Success = true,
                    Description = "",
                }
            };
            return spawnMiscObjectResponse;
        }

        private DespawnEntityResponse DespawnEntity(DespawnEntityRequest request) 
        {
            mainContext.Send(_ =>
            {
                lock (lockOnEntityDic)
                {
                    var instance = entityInstanceDic[request.Name];
                    entityInstanceDic.Remove(request.Name);
                    GameObject.Destroy(instance);
                }
            }, null);

            var despawnEntityResponse = new DespawnEntityResponse()
            {
                Result = new Result()
                {
                    Success = true,
                    Description = "",
                }
            };
            return despawnEntityResponse;
        }

        private AttachLidarSensorResponse AttachLidarSensor(AttachLidarSensorRequest request) 
        {
            var attachLidarSensorResponse = new AttachLidarSensorResponse()
            {
                Result = new Result()
                {
                    Success = false,
                    Description = "Not supported this request",
                }
            };
            return attachLidarSensorResponse;
        }

        private AttachDetectionSensorResponse AttachDetectionSensor(AttachDetectionSensorRequest request) 
        {
            var attachDetectionSensorResponse = new AttachDetectionSensorResponse()
            {
                Result = new Result()
                {
                    Success = false,
                    Description = "Not supported this request",
                }
            };
            return attachDetectionSensorResponse;
        }

        private AttachOccupancyGridSensorResponse AttachOccupancyGridSensor(AttachOccupancyGridSensorRequest request) 
        {
            var attachOccupancyGridSensorResponse = new AttachOccupancyGridSensorResponse()
            {
                Result = new Result()
                {
                    Success = false,
                    Description = "Not supported this request",
                }
            };
            return attachOccupancyGridSensorResponse;
        }
        
        private UpdateTrafficLightsResponse UpdateTrafficLights(UpdateTrafficLightsRequest request) 
        {
            var states = request.States;

            mainContext.Send(_ =>
            {
                resetModifiedTrafficLights();
                foreach (var state in states) {
                    foreach(var signal in state.TrafficLightStatus) {
                        var bulb = fromProto(signal);
                        trafficLights[state.Id].SetBulbData(bulb);
                        modifiedTrafficLights.Add(state.Id);
                    }
                }
            }, null);

            var updateTrafficLightsResponse = new UpdateTrafficLightsResponse()
            {
                Result = new Result()
                {
                    Success = true,
                    Description = "AWSIM does not support traffic light setting",
                }
            };
            return updateTrafficLightsResponse;
        }

        private AttachPseudoTrafficLightDetectorResponse UpdateTrafficLights(AttachPseudoTrafficLightDetectorRequest request) 
        {
            var attachPseudoTrafficLightDetectorResponse = new AttachPseudoTrafficLightDetectorResponse()
            {
                Result = new Result()
                {
                    Success = true,
                    Description = "AWSIM does not support traffic light setting",
                }
            };
            return attachPseudoTrafficLightDetectorResponse;
        }
        #endregion

        #region [Helpers]

        private static (Vector3 position, Quaternion rotation) GetUnityPose(GeometryMsgs.Pose rosPose)
        {
            var rosPosition = new Vector3((float)rosPose.Position.X, (float)rosPose.Position.Y, (float)rosPose.Position.Z) - Environment.Instance.MgrsOffsetPosition;
            var rosRotation = new Quaternion((float)rosPose.Orientation.X, (float)rosPose.Orientation.Y, (float)rosPose.Orientation.Z, (float)rosPose.Orientation.W);
            var unityPosition = ROS2Utility.RosToUnityPosition(rosPosition);
            var unityRotation = ROS2Utility.RosToUnityRotation(rosRotation);

            return (unityPosition, unityRotation);
        }

        private static GeometryMsgs.Pose GetRosPose(Transform unityPose)
        {
            var unityPosition = unityPose.position;
            var unityRotation = unityPose.rotation;
            var rosPosition = ROS2Utility.UnityToRosPosition(unityPosition) + Environment.Instance.MgrsOffsetPosition;
            var rosRotation = ROS2Utility.UnityToRosRotation(unityRotation);

            GeometryMsgs.Pose pose = new GeometryMsgs.Pose();
            pose.Position = new GeometryMsgs.Point();
            pose.Position.X = rosPosition.x;
            pose.Position.Y = rosPosition.y;
            pose.Position.Z = rosPosition.z;
            pose.Orientation = new GeometryMsgs.Quaternion();
            pose.Orientation.X = rosRotation.x;
            pose.Orientation.Y = rosRotation.y;
            pose.Orientation.Z = rosRotation.z;
            pose.Orientation.W = rosRotation.w;
            return pose;
        }

        private static TrafficSimulatorMsgs.ActionStatus GetEgoActionStatus(Vehicle instance, TrafficSimulatorMsgs.ActionStatus oldActionStatus) 
        {
            var actionStatus = new TrafficSimulatorMsgs.ActionStatus();
            actionStatus.CurrentAction = oldActionStatus.CurrentAction;
            var rosLinearVelocity = ROS2Utility.UnityToRosPosition(instance.LocalVelocity);
            actionStatus.Twist = new GeometryMsgs.Twist();
            actionStatus.Twist.Linear = new GeometryMsgs.Vector3();
            actionStatus.Twist.Linear.X = rosLinearVelocity.x;
            actionStatus.Twist.Linear.Y = rosLinearVelocity.y;
            actionStatus.Twist.Linear.Z = rosLinearVelocity.z;

            // instance.AngularVelocity seems to be in global coordinate system
            // But unless ego does not roll it should be mostly fine
            var rosAngularVelocity = ROS2Utility.UnityToRosPosition(instance.LocalAngularVelocity);
            actionStatus.Twist.Angular = new GeometryMsgs.Vector3();
            actionStatus.Twist.Angular.X = -rosAngularVelocity.x;
            actionStatus.Twist.Angular.Y = -rosAngularVelocity.y;
            actionStatus.Twist.Angular.Z = -rosAngularVelocity.z;

            var rosAccel = ROS2Utility.UnityToRosPosition(instance.LocalAcceleration);
            actionStatus.Accel = new GeometryMsgs.Accel();
            actionStatus.Accel.Linear = new GeometryMsgs.Vector3();
            actionStatus.Accel.Linear.X = rosAccel.x;
            actionStatus.Accel.Linear.Y = rosAccel.y;
            actionStatus.Accel.Linear.Z = rosAccel.z;

            var rosAngularAccel = ROS2Utility.UnityToRosPosition(instance.LocalAngularAcceleration);
            actionStatus.Accel.Angular = new GeometryMsgs.Vector3();
            actionStatus.Accel.Angular.X = -rosAngularAccel.x;
            actionStatus.Accel.Angular.Y = -rosAngularAccel.y;
            actionStatus.Accel.Angular.Z = -rosAngularAccel.z;

            // // In simple sensor simulator acceleration is set as jerk and it works
            // // It should most likely be fixed at some point
            actionStatus.LinearJerk = actionStatus.Accel.Linear.X;
            return actionStatus;
        }

        #endregion
    }
}
