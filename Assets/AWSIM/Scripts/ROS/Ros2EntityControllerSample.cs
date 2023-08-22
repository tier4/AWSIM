using System.Collections;
using System.Collections.Generic;
using System.Collections.Concurrent;
using UnityEngine;
using System;
using System.Linq;
using ROS2;
using Object = UnityEngine.Object;

namespace AWSIM
{
    public class Ros2EntityControllerSample : MonoBehaviour
    {
        public abstract class Entity
        {
            private bool _spawned;
            private string _assetKey;
            private string _uniqueId;
            private geometry_msgs.msg.Pose _pose;
            private double _yaw;
            protected double _linearSpeed;

            public Entity(string asset_key, string unique_id, Vector3 pose, float orientation)
            {
                _spawned = false;
                _assetKey = asset_key;
                _uniqueId = unique_id;
                _pose = new geometry_msgs.msg.Pose();
                _pose.Position.X = pose.z + Environment.Instance.MgrsOffsetPosition.x;
                _pose.Position.Y = -pose.x + Environment.Instance.MgrsOffsetPosition.y;
                _pose.Position.Z = pose.y + Environment.Instance.MgrsOffsetPosition.z;
                _pose.Orientation = ROS2Utility.RosQuaternionFromEuler(0, 0, -orientation);
                _yaw = -orientation;
                _linearSpeed = 0;
            }

            public entity_controller_msgs.msg.SpawnEntity Spawn()
            {
                entity_controller_msgs.msg.SpawnEntity spawn = new entity_controller_msgs.msg.SpawnEntity();
                spawn.Unique_id = _uniqueId;
                spawn.Asset_key = _assetKey;
                spawn.Pose = _pose;
                _spawned = true;
                return spawn;
            }

            public entity_controller_msgs.msg.UpdatePoseEntity UpdatePose(double x, double y, double yaw)
            {
                entity_controller_msgs.msg.UpdatePoseEntity update_pose = new entity_controller_msgs.msg.UpdatePoseEntity();
                update_pose.Unique_id = _uniqueId;
                update_pose.Pose = _pose;
                update_pose.Pose.Position.X = x;
                update_pose.Pose.Position.Y = y;
                update_pose.Pose.Orientation = ROS2Utility.RosQuaternionFromEuler(0, 0, yaw);
                _pose = update_pose.Pose;
                _yaw = yaw;
                return update_pose;
            }

            public entity_controller_msgs.msg.UpdatePoseEntity UpdatePoseMoveForward(double dt)
            {
                var distance = dt * _linearSpeed;
                double x = _pose.Position.X + distance * Math.Cos(_yaw);
                double y = _pose.Position.Y + distance * Math.Sin(_yaw);
                return UpdatePose(x, y, _yaw);
            }

            public entity_controller_msgs.msg.DespawnEntity Despawn()
            {
                entity_controller_msgs.msg.DespawnEntity despawn = new entity_controller_msgs.msg.DespawnEntity();
                despawn.Unique_id = _uniqueId;
                _spawned = false;
                return despawn;
            }

            public override string ToString()
            {
                return $"Key: {_assetKey}, ID: {_uniqueId}";
            }


        }

        public class VehicleEntity : Entity
        {
            public VehicleEntity(string asset_key, string unique_id, Vector3 pose, float orientation)
        : base(asset_key, unique_id, pose, orientation)
            {
                _linearSpeed = 13.0f;
            }
        }

        public class PedestrianEntity : Entity
        {
            public PedestrianEntity(string asset_key, string unique_id, Vector3 pose, float orientation)
        : base(asset_key, unique_id, pose, orientation)
            {
                _linearSpeed = 1.0f;
            }
        }

        public enum Step
        {
            Init,
            Spawn,
            MoveForward,
            Despawn,
        }

        [Serializable]
        public class Ros2Config
        {
            public string Topic;
            public QoSSettings Qos;
        }

        [Header("ROS2 config")]
        public Ros2Config SpawnConfig = new Ros2Config()
        {
            Topic = "awsim/entity_controller/spawn",
            Qos = new QoSSettings()
            {
                ReliabilityPolicy = ROS2.ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
                DurabilityPolicy = ROS2.DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
                HistoryPolicy = ROS2.HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
                Depth = 1,
            }
        };
        public Ros2Config UpdatePoseConfig = new Ros2Config()
        {
            Topic = "awsim/entity_controller/update_pose",
            Qos = new QoSSettings()
            {
                ReliabilityPolicy = ROS2.ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT,
                DurabilityPolicy = ROS2.DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
                HistoryPolicy = ROS2.HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
                Depth = 5,
            }
        };
        public Ros2Config DespawnConfig = new Ros2Config()
        {
            Topic = "awsim/entity_controller/despawn",
            Qos = new QoSSettings()
            {
                ReliabilityPolicy = ROS2.ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
                DurabilityPolicy = ROS2.DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
                HistoryPolicy = ROS2.HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
                Depth = 1,
            }
        };

        // Publishers
        private IPublisher<entity_controller_msgs.msg.SpawnEntity> spawnPublisher;
        private IPublisher<entity_controller_msgs.msg.UpdatePoseEntity> updatePosePublisher;
        private IPublisher<entity_controller_msgs.msg.DespawnEntity> despawnPublisher;

        // Containers
        private List<Entity> entities = new List<Entity>();
        Step currentStep = Step.Init, nextStep = Step.Init;
        bool stepStarted = false;

        void Start()
        {
            spawnPublisher = SimulatorROS2Node.CreatePublisher<entity_controller_msgs.msg.SpawnEntity>(SpawnConfig.Topic, SpawnConfig.Qos.GetQoSProfile());
            updatePosePublisher = SimulatorROS2Node.CreatePublisher<entity_controller_msgs.msg.UpdatePoseEntity>(UpdatePoseConfig.Topic, UpdatePoseConfig.Qos.GetQoSProfile());
            despawnPublisher = SimulatorROS2Node.CreatePublisher<entity_controller_msgs.msg.DespawnEntity>(DespawnConfig.Topic, DespawnConfig.Qos.GetQoSProfile());
        }

        private void Update()
        {
            if (!stepStarted)
            {
                switch (currentStep)
                {
                    case Step.Init:
                        {
                            StartCoroutine(Init());
                            stepStarted = true;
                            break;
                        }
                    case Step.Spawn:
                        {
                            StartCoroutine(Spawn());
                            stepStarted = true;
                            break;
                        }
                    case Step.MoveForward:
                        {
                            StartCoroutine(MoveForward());
                            stepStarted = true;
                            break;
                        }
                    case Step.Despawn:
                    default:
                        {
                            StartCoroutine(Despawn());
                            stepStarted = true;
                            break;
                        }
                }
            }

            if (stepStarted && nextStep != currentStep)
            {
                currentStep = nextStep;
                stepStarted = false;
            }
        }

        private IEnumerator Init()
        {
            if (entities.Count > 0)
                entities.Clear();

            // Group 1
            var firstGroupOrientation = -10.573f * Mathf.PI / 180f;
            entities.Add(new VehicleEntity("smallcar", "1", new Vector3(-461.459991f, -3.61436844f, -50.7299995f), firstGroupOrientation));
            entities.Add(new VehicleEntity("taxi", "2", new Vector3(-459.630005f, -3.68000007f, -60.2599983f), firstGroupOrientation));
            entities.Add(new VehicleEntity("van", "3", new Vector3(-457.220001f, -3.71000004f, -73.1600037f), firstGroupOrientation));
            entities.Add(new VehicleEntity("hatchback", "4", new Vector3(-458.170013f, -3.61436844f, -50.1199989f), firstGroupOrientation));
            entities.Add(new VehicleEntity("van", "5", new Vector3(-455.5f, -3.63000011f, -64.4400024f), firstGroupOrientation));
            entities.Add(new VehicleEntity("taxi", "6", new Vector3(-443.899994f, -3.79999995f, -127.279999f), firstGroupOrientation));


            // Group 2
            var secondGroupOrientation = 169.742844f * Mathf.PI / 180f;
            entities.Add(new VehicleEntity("taxi", "7", new Vector3(-463.529999f, -3.55999994f, 11.5500002f), secondGroupOrientation));
            entities.Add(new VehicleEntity("van", "8", new Vector3(-458.669995f, -3.61436844f, 2.51999998f), secondGroupOrientation));
            entities.Add(new VehicleEntity("smallcar", "9", new Vector3(-474.059998f, -3.74000001f, 69.7300034f), secondGroupOrientation));


            // Humans
            entities.Add(new PedestrianEntity("human_elegant", "10", new Vector3(-438.600006f, -3.22314072f, -35.9799995f), -8.565f * Mathf.PI / 180f));
            entities.Add(new PedestrianEntity("human_elegant", "11", new Vector3(-444.390015f, -3.02999997f, -14.6199999f), 171.617f * Mathf.PI / 180f));
            yield return new WaitForSeconds(1.0f);
            nextStep = Step.Spawn;
        }

        private IEnumerator Spawn()
        {
            foreach (Entity entity in entities)
            {
                entity_controller_msgs.msg.SpawnEntity spawn_msg = entity.Spawn();
                spawnPublisher.Publish(spawn_msg);
                yield return new WaitForSeconds(0.05f);
            }
            nextStep = Step.MoveForward;
        }

        private IEnumerator MoveForward(float duration = 15.0f, float dt = 0.05f)
        {
            var startTime = Time.fixedTime;
            while (Time.fixedTime - startTime < duration)
            {
                foreach (Entity entity in entities)
                {
                    entity_controller_msgs.msg.UpdatePoseEntity update_msg = entity.UpdatePoseMoveForward(dt);
                    updatePosePublisher.Publish(update_msg);
                }
                yield return new WaitForSeconds(dt);
            }
            nextStep = Step.Despawn;
        }

        private IEnumerator Despawn()
        {
            foreach (Entity entity in entities)
            {
                entity_controller_msgs.msg.DespawnEntity spawn_msg = entity.Despawn();
                despawnPublisher.Publish(spawn_msg);
                yield return new WaitForSeconds(0.05f);
            }
            nextStep = Step.Init;
        }
    }
}
