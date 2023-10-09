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
    /// Class to process client (Scanrio Simulator v2) requests.
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

        [Header("Time Source")]
        [SerializeField] private ExternalTimeSource timeSource = default;

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
            timeSource.Initialize();
            
            mainContext = SynchronizationContext.Current;

            fixedDeltaTime = Time.fixedDeltaTime;

            Time.timeScale = stepExecution? 0f : 1f;

            entityPrefabDic = new Dictionary<string, GameObject>(); 
            entityPrefabDic = entityPrefabs.ToDictionary(x => x.AssetKey, y => y.Prefab);

            entityInstanceDic = new Dictionary<string, GameObject>();
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

        #region [Unity Message = Fixed Update]

        private void FixedUpdate()
        {
            /*
            lock (lockOnFrameUpdate)
            {
                // Has the initialize request been received?
                if (!isInitialized)
                {
                    return;
                }

                if (stepExecution) 
                {
                    fixedUpdateCount++;

                    // Stop time when FixedUpdate() has been called a target number of times.
                    if (fixedUpdateCount > targetFixedUpdateCount)
                    {
                        isFixedUpdating = false;
                    }
                }
            }
            */
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

                //fixedUpdateCount = 0;
                //targetFixedUpdateCount = (int)(elapsedSec / fixedDeltaTime);

                // start time flow
                mainContext.Send(_ =>
                {
                    lock (lockOnFrameUpdate)
                    {
                        //isFixedUpdating = true;
                        Time.timeScale = realtimeFactor;
                    }
                }, null);
                
                // waiting
                // while (isFixedUpdating) { }

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
