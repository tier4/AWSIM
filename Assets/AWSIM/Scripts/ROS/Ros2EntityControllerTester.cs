using System.Collections;
using System.Collections.Generic;
using System.Collections.Concurrent;
using UnityEngine;
using System;
using System.Linq;
using ROS2;
using Object = UnityEngine.Object;

namespace AWSIM.Test
{
    using SpawnEntity = entity_controller_msgs.msg.SpawnEntity;
    using UpdatePoseEntity = entity_controller_msgs.msg.UpdatePoseEntity;
    using DespawnEntity = entity_controller_msgs.msg.DespawnEntity;
    using Pose = geometry_msgs.msg.Pose;
    using Quaternion = geometry_msgs.msg.Quaternion;

    public enum TestStep
    {
        Init,
        Spawn,
        MoveForward,
        Despawn,
    }

    public class Entity
    {
        public Quaternion QuaternionFromEuler(double roll, double pitch, double yaw)
        {
            double cy = Math.Cos(yaw * 0.5f);
            double sy = Math.Sin(yaw * 0.5f);
            double cp = Math.Cos(pitch * 0.5f);
            double sp = Math.Sin(pitch * 0.5f);
            double cr = Math.Cos(roll * 0.5f);
            double sr = Math.Sin(roll * 0.5f);

            Quaternion q = new Quaternion();
            q.W = cy * cp * cr + sy * sp * sr;
            q.X = cy * cp * sr - sy * sp * cr;
            q.Y = sy * cp * sr + cy * sp * cr;
            q.Z = sy * cp * cr - cy * sp * sr;
            return q;
        }

        private bool _spawned;
        private string _asset_key;
        private string _unique_id;
        private Pose _pose;
        private double _yaw;
        protected double _linear_speed;

        public Entity(string asset_key, string unique_id, Vector3 pose, float orientation)
        {
            _spawned = false;
            _asset_key = asset_key;
            _unique_id = unique_id;
            _pose = new Pose();
            _pose.Position.X = pose.z + Environment.Instance.MgrsOffsetPosition.x;
            _pose.Position.Y = -pose.x + Environment.Instance.MgrsOffsetPosition.y;
            _pose.Position.Z = pose.y + Environment.Instance.MgrsOffsetPosition.z;
            _pose.Orientation = QuaternionFromEuler(0, 0, -orientation);
            _yaw = -orientation;
            _linear_speed = 0;
        }

        public SpawnEntity Spawn()
        {
            SpawnEntity spawn = new SpawnEntity();
            spawn.Unique_id = _unique_id;
            spawn.Asset_key = _asset_key;
            spawn.Pose = _pose;
            _spawned = true;
            return spawn;
        }

        public UpdatePoseEntity UpdatePose(double x, double y, double yaw)
        {
            UpdatePoseEntity update_pose = new UpdatePoseEntity();
            update_pose.Unique_id = _unique_id;
            update_pose.Pose = _pose;
            update_pose.Pose.Position.X = x;
            update_pose.Pose.Position.Y = y;
            update_pose.Pose.Orientation = QuaternionFromEuler(0, 0, yaw);
            _pose = update_pose.Pose;
            _yaw = yaw;
            return update_pose;
        }

        public UpdatePoseEntity UpdatePoseMoveForward(double dt)
        {
            var distance = dt * _linear_speed;
            double x = _pose.Position.X + distance * Math.Cos(_yaw);
            double y = _pose.Position.Y + distance * Math.Sin(_yaw);
            return UpdatePose(x, y, _yaw);
        }

        public DespawnEntity Despawn()
        {
            DespawnEntity despawn = new DespawnEntity();
            despawn.Unique_id = _unique_id;
            _spawned = false;
            return despawn;
        }

        public override string ToString()
        {
            return $"Key: {_asset_key}, ID: {_unique_id}";
        }
    }

    public class VehicleEntity : Entity
    {
        public VehicleEntity(string asset_key, string unique_id, Vector3 pose, float orientation)
    : base(asset_key, unique_id, pose, orientation)
        {
            _linear_speed = 13.0f;
        }
    }

    public class PedestrianEntity : Entity
    {
        public PedestrianEntity(string asset_key, string unique_id, Vector3 pose, float orientation)
    : base(asset_key, unique_id, pose, orientation)
        {
            _linear_speed = 1.0f;
        }
    }

    public class Ros2EntityControllerTester : MonoBehaviour
    {
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
        TestStep currentStep = TestStep.Init, nextStep = TestStep.Init;
        bool stepStarted = false;

        void Start()
        {
            spawnPublisher = SimulatorROS2Node.CreatePublisher<SpawnEntity>(SpawnConfig.Topic, SpawnConfig.Qos.GetQoSProfile());
            updatePosePublisher = SimulatorROS2Node.CreatePublisher<UpdatePoseEntity>(UpdatePoseConfig.Topic, UpdatePoseConfig.Qos.GetQoSProfile());
            despawnPublisher = SimulatorROS2Node.CreatePublisher<DespawnEntity>(DespawnConfig.Topic, DespawnConfig.Qos.GetQoSProfile());
        }

        private void FixedUpdate()
        {
            if (!stepStarted)
            {
                Debug.Log("Step " + currentStep + " done:" + stepStarted);
                switch (currentStep)
                {
                    case TestStep.Init:
                        {
                            Init();
                            stepStarted = true;
                            break;
                        }
                    case TestStep.Spawn:
                        {
                            StartCoroutine(Spawn());
                            stepStarted = true;
                            break;
                        }
                    case TestStep.MoveForward:
                        {
                            StartCoroutine(MoveForward());
                            stepStarted = true;
                            break;
                        }
                    case TestStep.Despawn:
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

        private void Init()
        {
            if (entities.Count > 0)
                entities.Clear();

            // Group 1
            var firstGroupOrientation = -10.573f * Mathf.PI / 180f;
            entities.Add(new VehicleEntity("smallcar", "1", new Vector3(-461.459991f, -3.61436844f, -50.7299995f), firstGroupOrientation));
            entities.Add(new VehicleEntity("taxi", "2", new Vector3(-460.420013f, -3.61436844f, -56.0200005f), firstGroupOrientation));
            entities.Add(new VehicleEntity("van", "3", new Vector3(-459.380005f, -3.61436844f, -61.5900002f), firstGroupOrientation));
            entities.Add(new VehicleEntity("hatchback", "4", new Vector3(-458.170013f, -3.61436844f, -50.1199989f), firstGroupOrientation));
            entities.Add(new VehicleEntity("van", "5", new Vector3(-457.140015f, -3.61436844f, -55.6399994f), firstGroupOrientation));
            entities.Add(new VehicleEntity("hatchback", "6", new Vector3(-456.130005f, -3.61436844f, -61.0600014f), firstGroupOrientation));

            // Group 2
            var secondGroupOrientation = 169.742844f * Mathf.PI / 180f;
            entities.Add(new VehicleEntity("taxi", "7", new Vector3(-462.809998f, -3.61436844f, 7.57000017f), secondGroupOrientation));
            entities.Add(new VehicleEntity("hatchback", "8", new Vector3(-461.559998f, -3.61436844f, 1.82000005f), secondGroupOrientation));
            entities.Add(new VehicleEntity("van", "9", new Vector3(-458.669995f, -3.61436844f, 2.51999998f), secondGroupOrientation));

            // Humans
            entities.Add(new PedestrianEntity("human_elegant", "10", new Vector3(-438.600006f, -3.22314072f, -35.9799995f), -2.874f * Mathf.PI / 180f));
            entities.Add(new PedestrianEntity("human_elegant", "11", new Vector3(-444.390015f, -3.02999997f, -14.6199999f), 162.617f * Mathf.PI / 180f));
            nextStep = TestStep.Spawn;
        }

        private IEnumerator Spawn()
        {
            foreach (Entity entity in entities)
            {
                SpawnEntity spawn_msg = entity.Spawn();
                spawnPublisher.Publish(spawn_msg);
                yield return new WaitForSeconds(0.05f);
            }
            nextStep = TestStep.MoveForward;
        }

        private IEnumerator MoveForward(float duration = 20.0f, float dt = 0.01f)
        {
            var startTime = Time.fixedTime;
            while (Time.fixedTime - startTime < duration)
            {
                yield return new WaitForFixedUpdate();
                foreach (Entity entity in entities)
                {
                    UpdatePoseEntity update_msg = entity.UpdatePoseMoveForward(Time.deltaTime);
                    updatePosePublisher.Publish(update_msg);
                }
            }
            nextStep = TestStep.Despawn;
        }

        private IEnumerator Despawn()
        {
            foreach (Entity entity in entities)
            {
                DespawnEntity spawn_msg = entity.Despawn();
                despawnPublisher.Publish(spawn_msg);
                yield return new WaitForSeconds(0.05f);
            }
            nextStep = TestStep.Init;
        }
    }
}
