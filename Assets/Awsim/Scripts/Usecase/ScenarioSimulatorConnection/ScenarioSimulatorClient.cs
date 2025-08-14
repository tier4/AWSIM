// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Linq;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using ZeroMQ;
using Google.Protobuf;
using SimulationApiSchema;
using Awsim.Common;
using Awsim.Entity;

namespace Awsim.Usecase.ScenarioSimulatorConnection
{
    public class ScenarioSimulatorClient : MonoBehaviour
    {
        [SerializeField] string _serverResponseAddress = "tcp://127.0.0.1:8080";

        [Header("Entities")]
        [SerializeField] Entity.TrafficLight[] _trafficLightsInScene;
        Thread _thread = default;

        [Serializable]
        private class EntityPrefab
        {
            public string AssetKey;
            public GameObject Prefab;
        }

        [Header("Entities")]
        [SerializeField] EntityPrefab[] _entityPrefabs;
        [SerializeField] Transform _entitesRoot;
        [SerializeField] FollowCamera _egoFollowCamera;

        [Header("Execution Type")]
        [SerializeField] bool _stepExecution = false;
        [Range(0f, 1f)]
        [SerializeField] float _stepDurationInPercentage = 0.5f;

        Dictionary<string, GameObject> _entityPrefabDic = new();
        Dictionary<string, GameObject> _entityInstanceDic = new();
        Dictionary<long, Entity.TrafficLight> trafficLights;
        List<long> modifiedTrafficLights;
        SynchronizationContext mainContext;
        ExternalTimeSource _timeSource = default;

        static readonly object _lockObject = new();
        static readonly object _lockOnEntityDic = new();
        static readonly object _lockOnFrameUpdate = new();

        float realtimeFactor = 0f;
        float stepTime = 0f;

        public void Initialize()
        {
            foreach (var light in _trafficLightsInScene)
                light.Initialize();

            InitializeProcessor();
            StartServerResponseThread();
        }

        void OnDestroy()
        {
            StopServerResponseThread();
            Dispose();
        }

        void OnApplicationQuit()
        {
            StopServerResponseThread();
            Dispose();
        }

        public void OnUpdate()
        {
            foreach (var e in trafficLights)
                e.Value.OnUpdate();

            _egoFollowCamera.OnUpdate();

            foreach (var e in _entityInstanceDic)
            {
                if (e.Value.TryGetComponent(out Pedestrian npcPedestrian))
                {
                    npcPedestrian.OnUpdate();
                }
                else if (e.Value.TryGetComponent(out PoseVehicle npcVehicle))
                {
                    npcVehicle.OnUpdate();
                }
                else if (e.Value.TryGetComponent(out IScenarioEgoVehicle egoVehicle))
                {
                    egoVehicle.OnUpdate();
                }
            }
        }

        public void OnFixedUpdate()
        {
            foreach (var e in _entityInstanceDic)
            {
                if (e.Value.TryGetComponent(out Pedestrian npcPedestrian))
                {
                    npcPedestrian.OnFixedUpdate();
                }
                else if (e.Value.TryGetComponent(out PoseVehicle npcVehicle))
                {
                    npcVehicle.OnFixedUpdate();
                }
                else if (e.Value.TryGetComponent(out IScenarioEgoVehicle egoVehicle))
                {
                    egoVehicle.OnFixedUpdate();
                }
            }
        }

        public void DisposeClient()
        {
            StopServerResponseThread();
            Dispose();
        }

        void InitializeProcessor()
        {
            _timeSource = AwsimRos2Node.GetTimeSource() as ExternalTimeSource;
            if (_timeSource == null)
            {
                Debug.LogError("Scenario Simulator requires time source of type SS2. Check if TimeSourceSelector is on the scene and SS2 is the selected as the Time Source");
            }
            _timeSource.Initialize();
            mainContext = SynchronizationContext.Current;
            Time.timeScale = _stepExecution ? 0f : 1f;

            _entityPrefabDic = _entityPrefabs.ToDictionary(x => x.AssetKey, y => y.Prefab);
            _entityInstanceDic.Clear();

            PreprocessTrafficLights();
            ResetAllTrafficLights();
            _egoFollowCamera.Initialize();
        }

        void Dispose()
        {
            _entityPrefabDic?.Clear();
            _entityInstanceDic?.Clear();
            Time.timeScale = 1f;
            mainContext = null;
            _timeSource?.Dispose();
        }

        void StartServerResponseThread()
        {
            _thread = new Thread(() =>
            {
                using (ZContext context = new ZContext())
                {
                    using (ZSocket responseSocket = new ZSocket(context, ZSocketType.REP))
                    {
                        Debug.Log("[ZMQ] Binding to: " + _serverResponseAddress);
                        responseSocket.Bind(_serverResponseAddress);

                        ZPollItem pollItem = ZPollItem.CreateReceiver();
                        ZError zError;
                        ZMessage zMessage;

                        while (true)
                        {
                            if (responseSocket.PollIn(pollItem, out zMessage, out zError))
                            {
                                if (zMessage != null)
                                {
                                    for (int i = 0; i < zMessage.Count; i++)
                                    {
                                        byte[] buffer = new byte[zMessage[i].Length];
                                        zMessage.PopBytes(buffer, 0, buffer.Length);

                                        try
                                        {
                                            SimulationRequest request = SimulationRequest.Parser.ParseFrom(buffer);
                                            SimulationResponse response = Process(request);

                                            byte[] responseBytes = response.ToByteArray();
                                            if (!responseSocket.SendBytes(responseBytes, 0, responseBytes.Length))
                                            {
                                                Debug.LogWarning("[ZMQ ERROR] Failed to send a response.");
                                            }
                                        }
                                        catch (Exception ex)
                                        {
                                            Debug.LogError("[ZMQ ERROR] Failed to handle message: " + ex.Message);
                                        }
                                    }
                                }
                            }
                            else
                            {
                                if (zError == ZError.ETERM)
                                {
                                    Debug.LogError("[ZMQ ERROR] ZMQ context was terminated.");
                                }
                                else
                                {
                                    Debug.LogWarning("[ZMQ ERROR] " + zError);
                                }
                            }
                        }
                    }
                }
            });
            _thread.Name = "ServerResponse";
            _thread.Start();
        }


        void StopServerResponseThread()
        {
            _thread?.Abort();
            _thread = null;
        }

        void PreprocessTrafficLights()
        {
            modifiedTrafficLights = new List<long>();
            trafficLights = new Dictionary<long, Entity.TrafficLight>();
            var trafficLightObjects = FindObjectsByType<LaneletTrafficLight>(FindObjectsSortMode.InstanceID);
            for (int i = 0; i < trafficLightObjects.Length; i++)
            {
                LaneletTrafficLight.TrafficLightLaneletId laneletId = trafficLightObjects[i].LaneletId;
                GameObject obj = trafficLightObjects[i].gameObject;
                Entity.TrafficLight tl = obj.GetComponent<Entity.TrafficLight>();
                if (tl != null && laneletId != null)
                {
                    trafficLights.Add(laneletId.wayId, tl);
                }
            }
        }

        void ResetSingleTraffcLight(Entity.TrafficLight light)
        {
            light.SetBulbData(new Entity.TrafficLight.BulbData(Entity.TrafficLight.BulbType.GreenBulb, Entity.TrafficLight.BulbColor.Green, Entity.TrafficLight.BulbStatus.SolidOff));
            light.SetBulbData(new Entity.TrafficLight.BulbData(Entity.TrafficLight.BulbType.RedBulb, Entity.TrafficLight.BulbColor.Red, Entity.TrafficLight.BulbStatus.SolidOff));
            light.SetBulbData(new Entity.TrafficLight.BulbData(Entity.TrafficLight.BulbType.YellowBulb, Entity.TrafficLight.BulbColor.Yellow, Entity.TrafficLight.BulbStatus.SolidOff));
        }

        void ResetModifiedTrafficLights()
        {
            foreach (var trafficLightId in modifiedTrafficLights)
            {
                ResetSingleTraffcLight(trafficLights[trafficLightId]);
            }
            modifiedTrafficLights.Clear();
        }

        void ResetAllTrafficLights()
        {
            foreach (var light in trafficLights)
            {
                ResetSingleTraffcLight(light.Value);
            }
        }

        Entity.TrafficLight.BulbData FromProto(SimulationApiSchema.TrafficLight proto)
        {
            var t = new Entity.TrafficLight.BulbType();
            switch (proto.Shape)
            {
                case SimulationApiSchema.TrafficLight.Types.Shape.Circle: t = Entity.TrafficLight.BulbType.AnyCircleBulb; break;
                case SimulationApiSchema.TrafficLight.Types.Shape.LeftArrow: t = Entity.TrafficLight.BulbType.LeftArrowBulb; break;
                case SimulationApiSchema.TrafficLight.Types.Shape.RightArrow: t = Entity.TrafficLight.BulbType.RightArrowBulb; break;
                case SimulationApiSchema.TrafficLight.Types.Shape.UpArrow: t = Entity.TrafficLight.BulbType.UpArrowBulb; break;
                case SimulationApiSchema.TrafficLight.Types.Shape.DownArrow: t = Entity.TrafficLight.BulbType.DownArrowBulb; break;
                case SimulationApiSchema.TrafficLight.Types.Shape.DownLeftArrow: t = Entity.TrafficLight.BulbType.DownLeftArrowBulb; break;
                case SimulationApiSchema.TrafficLight.Types.Shape.DownRightArrow: t = Entity.TrafficLight.BulbType.DownRightArrowBulb; break;
                case SimulationApiSchema.TrafficLight.Types.Shape.Cross: t = Entity.TrafficLight.BulbType.CrossBulb; break;
            }

            var c = new Entity.TrafficLight.BulbColor();
            switch (proto.Color)
            {
                case SimulationApiSchema.TrafficLight.Types.Color.Red: c = Entity.TrafficLight.BulbColor.Red; t = Entity.TrafficLight.BulbType.RedBulb; break;
                case SimulationApiSchema.TrafficLight.Types.Color.Amber: c = Entity.TrafficLight.BulbColor.Yellow; t = Entity.TrafficLight.BulbType.YellowBulb; break;
                case SimulationApiSchema.TrafficLight.Types.Color.Green: c = Entity.TrafficLight.BulbColor.Green; t = Entity.TrafficLight.BulbType.GreenBulb; break;
                case SimulationApiSchema.TrafficLight.Types.Color.White: c = Entity.TrafficLight.BulbColor.White; break;
            }

            var s = new Entity.TrafficLight.BulbStatus();
            switch (proto.Status)
            {
                case SimulationApiSchema.TrafficLight.Types.Status.SolidOff: s = Entity.TrafficLight.BulbStatus.SolidOff; break;
                case SimulationApiSchema.TrafficLight.Types.Status.SolidOn: s = Entity.TrafficLight.BulbStatus.SolidOn; break;
                case SimulationApiSchema.TrafficLight.Types.Status.Flashing: s = Entity.TrafficLight.BulbStatus.Frashing; break;
            }
            return new Entity.TrafficLight.BulbData(t, c, s);
        }

        SimulationResponse Process(SimulationRequest request)
        {
            SimulationResponse response = new SimulationResponse();

            switch (request.RequestCase)
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
                case SimulationRequest.RequestOneofCase.UpdateStepTime:
                    response.UpdateStepTime = UpdateStepTime(request.UpdateStepTime);
                    break;
                default:
                    Debug.Log("Case " + request.RequestCase.ToString() + " not yet supported");
                    break;
            }

            return response;

        }

        InitializeResponse Initialize(InitializeRequest request)
        {
            realtimeFactor = (float)request.RealtimeFactor;
            stepTime = (float)request.StepTime;
            if (_stepExecution)
            {
                realtimeFactor *= (1f / _stepDurationInPercentage);
            }

            BuiltinInterfaces.Time currentRosTime = request.InitializeRosTime;
            _timeSource.SetTime(currentRosTime.Sec, currentRosTime.Nanosec);

            mainContext.Send(_ =>
            {
                lock (_lockOnFrameUpdate)
                {
                    Time.timeScale = _stepExecution ? 0f : realtimeFactor;
                }
            }, null);

            var initializeResponse = new InitializeResponse()
            {
                Result = new Result()
                {
                    Success = true,
                    Description = "AWSIM initialized",
                }
            };

            return initializeResponse;
        }

        UpdateFrameResponse UpdateFrame(UpdateFrameRequest request)
        {
            BuiltinInterfaces.Time currentRosTime = request.CurrentRosTime;
            _timeSource.SetTime(currentRosTime.Sec, currentRosTime.Nanosec);

            if (_stepExecution)
            {
                mainContext.Send(_ =>
                {
                    lock (_lockOnFrameUpdate)
                    {
                        Time.timeScale = realtimeFactor;
                    }
                }, null);

                int waitTime = Mathf.CeilToInt((float)(stepTime * 1000.0 * _stepDurationInPercentage));
                Thread.Sleep(waitTime);

                mainContext.Send(_ =>
                {
                    lock (_lockOnFrameUpdate)
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

        UpdateEntityStatusResponse UpdateEntityStatus(UpdateEntityStatusRequest request)
        {
            var updateEntityStatusResponse = new UpdateEntityStatusResponse();

            mainContext.Send(_ =>
            {
                for (int i = 0; i < request.Status.Count; i++)
                {
                    var reqStatus = request.Status[i];
                    var status = new UpdatedEntityStatus();
                    var unityPose = GetUnityPose(reqStatus.Pose);
                    var instance = _entityInstanceDic[reqStatus.Name];
                    if (instance.TryGetComponent(out PoseVehicle vehicle))
                    {
                        vehicle.PoseInput = unityPose;
                        status.Name = reqStatus.Name;
                        status.Pose = reqStatus.Pose;
                        status.ActionStatus = reqStatus.ActionStatus;
                        
                    }
                    else if (instance.TryGetComponent(out Pedestrian pedestrian))
                    {
                        pedestrian.PoseInput = unityPose;
                        status.Name = reqStatus.Name;
                        status.Pose = reqStatus.Pose;
                        status.ActionStatus = reqStatus.ActionStatus;
                        
                    }
                    else if (instance.TryGetComponent(out AccelVehicle ego_vehicle))
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
                    lock (_lockObject)
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

        SpawnVehicleEntityResponse SpawnVehicleEntity(SpawnVehicleEntityRequest request)
        {
            var asset_key = request.AssetKey;
            var pose = request.Pose;

            mainContext.Send(_ =>
            {
                lock (_lockOnEntityDic)
                {
                    var unityPose = GetUnityPose(pose);
                    var prefab = _entityPrefabDic[asset_key];
                    var instance = GameObject.Instantiate(prefab, unityPose.position, unityPose.rotation, _entitesRoot);
                    _entityInstanceDic.Add(request.Parameters.Name, instance);

                    if (instance.TryGetComponent<PoseVehicle>(out var npcVehicle))
                    {
                        npcVehicle.Initialize();
                    }
                    else if (instance.TryGetComponent<Pedestrian>(out var pedestrian))
                    {
                        pedestrian.Initialize();
                    }
                    else if (request.IsEgo)
                    {
                        _egoFollowCamera.Target = instance.transform;
                        var vehicle = instance.GetComponent<IScenarioEgoVehicle>();
                        vehicle.Initialize();
                    }
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

        SpawnPedestrianEntityResponse SpawnPedestrianEntity(SpawnPedestrianEntityRequest request)
        {
            var asset_key = request.AssetKey;
            var pose = request.Pose;

            mainContext.Send(_ =>
            {
                lock (_lockOnEntityDic)
                {
                    var unityPose = GetUnityPose(pose);
                    var prefab = _entityPrefabDic[asset_key];
                    var instance = GameObject.Instantiate(prefab, unityPose.position, unityPose.rotation, _entitesRoot);
                    _entityInstanceDic.Add(request.Parameters.Name, instance);

                    var npcPedestrian = instance.GetComponent<Pedestrian>();

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

        SpawnMiscObjectEntityResponse SpawnMiscObjectEntity(SpawnMiscObjectEntityRequest request)
        {
            var asset_key = request.AssetKey;
            var pose = request.Pose;
            
            mainContext.Send(_ =>
            {
                lock (_lockOnEntityDic)
                {
                    var unityPose = GetUnityPose(pose);
                    var prefab = _entityPrefabDic[asset_key];
                    var instance = GameObject.Instantiate(prefab, unityPose.position, unityPose.rotation, _entitesRoot);
                    _entityInstanceDic.Add(request.Parameters.Name, instance);
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

        DespawnEntityResponse DespawnEntity(DespawnEntityRequest request)
        {
            mainContext.Send(_ =>
            {
                lock (_lockOnEntityDic)
                {
                    var instance = _entityInstanceDic[request.Name];
                    _entityInstanceDic.Remove(request.Name);
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

        AttachLidarSensorResponse AttachLidarSensor(AttachLidarSensorRequest request)
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

        AttachDetectionSensorResponse AttachDetectionSensor(AttachDetectionSensorRequest request)
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

        AttachOccupancyGridSensorResponse AttachOccupancyGridSensor(AttachOccupancyGridSensorRequest request)
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

        UpdateTrafficLightsResponse UpdateTrafficLights(UpdateTrafficLightsRequest request)
        {
            var states = request.States;

            mainContext.Send(_ =>
            {
                ResetModifiedTrafficLights();
                foreach (var state in states)
                {
                    foreach (var signal in state.TrafficLightStatus)
                    {
                        var bulb = FromProto(signal);
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

        AttachPseudoTrafficLightDetectorResponse UpdateTrafficLights(AttachPseudoTrafficLightDetectorRequest request)
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

        UpdateStepTimeResponse UpdateStepTime(UpdateStepTimeRequest request)
        {
            stepTime = (float)request.SimulationStepTime;
            var updateStepTimeResponse = new UpdateStepTimeResponse()
            {
                Result = new Result()
                {
                    Success = true,
                    Description = "Updated step time in AWSIM"
                }
            };
            return updateStepTimeResponse;
        }

        static Pose GetUnityPose(GeometryMsgs.Pose rosPose)
        {
            var rosPosition = new Vector3((float)rosPose.Position.X, (float)rosPose.Position.Y, (float)rosPose.Position.Z) - MgrsPosition.Instance.Mgrs.Position;
            var rosRotation = new Quaternion((float)rosPose.Orientation.X, (float)rosPose.Orientation.Y, (float)rosPose.Orientation.Z, (float)rosPose.Orientation.W);
            var unityPosition = Ros2Utility.Ros2ToUnityPosition(rosPosition);
            var unityRotation = Ros2Utility.Ros2ToUnityQuaternion(rosRotation);

            return new Pose(unityPosition, unityRotation);
        }

        static GeometryMsgs.Pose GetRosPose(Transform unityPose)
        {
            var unityPosition = unityPose.position;
            var unityRotation = unityPose.rotation;
            var rosPosition = Ros2Utility.UnityToRos2Position(unityPosition) + MgrsPosition.Instance.Mgrs.Position;
            var rosRotation = Ros2Utility.UnityToRosRotation(unityRotation);

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

        static TrafficSimulatorMsgs.ActionStatus GetEgoActionStatus(AccelVehicle instance, TrafficSimulatorMsgs.ActionStatus oldActionStatus)
        {
            instance.Initialize();

            var actionStatus = new TrafficSimulatorMsgs.ActionStatus();
            actionStatus.CurrentAction = oldActionStatus.CurrentAction;
            var rosLinearVelocity = Ros2Utility.UnityToRos2Position(instance.LocalVelocity);
            actionStatus.Twist = new GeometryMsgs.Twist();
            actionStatus.Twist.Linear = new GeometryMsgs.Vector3();
            actionStatus.Twist.Linear.X = rosLinearVelocity.x;
            actionStatus.Twist.Linear.Y = rosLinearVelocity.y;
            actionStatus.Twist.Linear.Z = rosLinearVelocity.z;

            var rosAngularVelocity = Ros2Utility.UnityToRos2Position(instance.LocalAngularVelocity);
            actionStatus.Twist.Angular = new GeometryMsgs.Vector3();
            actionStatus.Twist.Angular.X = -rosAngularVelocity.x;
            actionStatus.Twist.Angular.Y = -rosAngularVelocity.y;
            actionStatus.Twist.Angular.Z = -rosAngularVelocity.z;

            var rosAccel = Ros2Utility.UnityToRos2Position(instance.LocalAcceleration);
            actionStatus.Accel = new GeometryMsgs.Accel();
            actionStatus.Accel.Linear = new GeometryMsgs.Vector3();
            actionStatus.Accel.Linear.X = rosAccel.x;
            actionStatus.Accel.Linear.Y = rosAccel.y;
            actionStatus.Accel.Linear.Z = rosAccel.z;

            var rosAngularAccel = Ros2Utility.UnityToRos2Position(instance.LocalAngularAcceleration);
            actionStatus.Accel.Angular = new GeometryMsgs.Vector3();
            actionStatus.Accel.Angular.X = -rosAngularAccel.x;
            actionStatus.Accel.Angular.Y = -rosAngularAccel.y;
            actionStatus.Accel.Angular.Z = -rosAngularAccel.z;

            actionStatus.LinearJerk = actionStatus.Accel.Linear.X;
            return actionStatus;
        }
    }
}
