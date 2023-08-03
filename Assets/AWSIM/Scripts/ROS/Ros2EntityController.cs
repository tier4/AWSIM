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

    public abstract class Entity
    {
        public string UniqueID { get; private set; }
        public string AssetKey { get; private set; }
        public GameObject gameObject;

        public Entity(string uniqueId, string assetKey, GameObject obj)
        {
            UniqueID = uniqueId;
            AssetKey = assetKey;
            gameObject = obj;
        }

        public abstract void SetPose(Vector3 newPosition, Quaternion newRotation);

        public override string ToString()
        {
            return $"UniqueID: {UniqueID}, AssetKey: {AssetKey}";
        }

    }

    public class VehicleEntity : Entity
    {
        private NPCVehicle vehicle;

        public VehicleEntity(string uniqueId, string assetKey, GameObject obj)
            : base(uniqueId, assetKey, obj)
        {
            vehicle = obj.GetComponent<NPCVehicle>();
        }

        public override void SetPose(Vector3 newPosition, Quaternion newRotation)
        {
            vehicle.SetPosition(newPosition);
            vehicle.SetRotation(newRotation);
        }
    }

    public class PedestrianEntity : Entity
    {
        private NPCPedestrian pedestrian;

        public PedestrianEntity(string uniqueId, string assetKey, GameObject obj)
            : base(uniqueId, assetKey, obj)
        {
            pedestrian = obj.GetComponent<NPCPedestrian>();
        }

        public override void SetPose(Vector3 newPosition, Quaternion newRotation)
        {
            pedestrian.SetPosition(newPosition);
            pedestrian.SetRotation(newRotation);
        }
    }

    public class ObjectEntity : Entity
    {
        public Rigidbody rigidBody;

        public ObjectEntity(string uniqueId, string assetKey, GameObject obj)
            : base(uniqueId, assetKey, obj)
        {
            rigidBody = obj.GetComponent<Rigidbody>();
        }

        public override void SetPose(Vector3 newPosition, Quaternion newRotation)
        {
            rigidBody.MovePosition(newPosition);
            rigidBody.MoveRotation(newRotation);
        }
    }

    public class Ros2EntityController : MonoBehaviour
    {
        [Serializable]
        public class EntityPrefab
        {
            public string AssetKey;
            public GameObject prefab;
        }

        [Serializable]
        public class Ros2Config
        {
            public string Topic;
            public QoSSettings Qos;
        }

        [Header("Entity")]
        public EntityPrefab[] npcVehiclePrefabs;
        public EntityPrefab[] npcPedestrianPrefabs;
        public EntityPrefab[] objectsPrefabs;

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

        // Subscribers
        private ISubscription<entity_controller_msgs.msg.SpawnEntity> spawnSubscriber;
        private ISubscription<entity_controller_msgs.msg.UpdatePoseEntity> updatePoseSubscriber;
        private ISubscription<entity_controller_msgs.msg.DespawnEntity> despawnSubscriber;

        // Spawned entities
        private List<Entity> entities = new List<Entity>();
        // Command execution queue in main-thread (for callbacks)
        private ConcurrentQueue<Action> concurrentQueue = new ConcurrentQueue<Action>();

        void Start()
        {
            // Prevent duplicate asset keys
            var npcVehicleKeys = npcVehiclePrefabs.Select(x => x.AssetKey);
            var npcPedestrianKeys = npcPedestrianPrefabs.Select(x => x.AssetKey);
            var objectsKeys = objectsPrefabs.Select(x => x.AssetKey);
            var allkeys = npcVehicleKeys.Concat(npcPedestrianKeys).Concat(objectsKeys);
            bool duplicatedAssetKey = allkeys.GroupBy(x => x).Where(x => x.Count() > 1).Count() > 0;
            if (duplicatedAssetKey)
            {
                Debug.LogError("Duplicate asset key");
                return;
            }

            spawnSubscriber
                = SimulatorROS2Node.CreateSubscription<entity_controller_msgs.msg.SpawnEntity>(
                    SpawnConfig.Topic, msg =>
                    {
                        concurrentQueue.Enqueue(() => Spawn(msg.Asset_key, msg.Unique_id,
                                                            ROS2Utility.RosMGRSToUnityPosition(msg.Pose.Position),
                                                            ROS2Utility.RosToUnityRotation(msg.Pose.Orientation)));
                    }, SpawnConfig.Qos.GetQoSProfile());

            updatePoseSubscriber
                = SimulatorROS2Node.CreateSubscription<entity_controller_msgs.msg.UpdatePoseEntity>(
                    UpdatePoseConfig.Topic, msg =>
                    {
                        concurrentQueue.Enqueue(() => UpdatePose(msg.Unique_id,
                                                                ROS2Utility.RosMGRSToUnityPosition(msg.Pose.Position),
                                                                ROS2Utility.RosToUnityRotation(msg.Pose.Orientation)));
                    }, UpdatePoseConfig.Qos.GetQoSProfile());

            despawnSubscriber = SimulatorROS2Node.CreateSubscription<entity_controller_msgs.msg.DespawnEntity>(
                    DespawnConfig.Topic, msg =>
                    {
                        concurrentQueue.Enqueue(() => Despawn(msg.Unique_id));
                    }, DespawnConfig.Qos.GetQoSProfile());

        }

        private void FixedUpdate()
        {
            while (concurrentQueue.Count > 0)
            {
                if (concurrentQueue.TryDequeue(out Action action))
                {
                    action.Invoke();
                }
            }
        }

        public void Spawn(string assetKey, string uniqueId, Vector3 spawnPosition, Quaternion spawnRotation)
        {
            if (entities.Find(entity => entity.UniqueID == uniqueId) != null)
            {
                Debug.LogError($"Cannot spawn entity UniqueID=='{uniqueId}' - it already exists.");
                return;
            }

            var npcVehiclePrefab = npcVehiclePrefabs.FirstOrDefault(obj => obj.AssetKey == assetKey);
            var npcPedestrianPrefab = npcPedestrianPrefabs.FirstOrDefault(obj => obj.AssetKey == assetKey);
            var objectsPrefab = objectsPrefabs.FirstOrDefault(obj => obj.AssetKey == assetKey);

            GameObject obj = null;
            if (npcVehiclePrefab != null)
            {
                obj = Object.Instantiate(npcVehiclePrefab.prefab, spawnPosition, spawnRotation);
                obj.name = obj.name + "_" + uniqueId;
                obj.transform.parent = this.transform;
                entities.Add(new VehicleEntity(uniqueId, assetKey, obj));
            }
            if (npcPedestrianPrefab != null)
            {
                obj = Object.Instantiate(npcPedestrianPrefab.prefab, spawnPosition, spawnRotation);
                obj.name = obj.name + "_" + uniqueId;
                obj.transform.parent = this.transform;
                entities.Add(new PedestrianEntity(uniqueId, assetKey, obj));
            }
            if (objectsPrefab != null)
            {
                obj = Object.Instantiate(objectsPrefab.prefab, spawnPosition, spawnRotation);
                obj.name = obj.name + "_" + uniqueId;
                obj.transform.parent = this.transform;
                entities.Add(new ObjectEntity(uniqueId, assetKey, obj));
            }

            if (obj == null)
            {
                Debug.LogError($"Cannot spawn entity AssetKey=='{assetKey}' - such an entity does not exist.");
                return;
            }
        }

        public void UpdatePose(string uniqueId, Vector3 newPosition, Quaternion newRotation)
        {
            Entity targetEntity = entities.Find(entity => entity.UniqueID == uniqueId);
            if (targetEntity == null)
            {
                Debug.LogError($"Cannot update entity UniqueID=='{uniqueId}' - such an entity does not exist.");
                return;
            }
            targetEntity.SetPose(newPosition, newRotation);
        }

        public void Despawn(string uniqueId)
        {
            Entity targetEntity = entities.Find(entity => entity.UniqueID == uniqueId);
            if (targetEntity == null)
            {
                Debug.LogError($"Cannot despawn entity UniqueID=='{uniqueId}' - such an entity does not exist.");
                return;
            }
            Object.DestroyImmediate(targetEntity.gameObject);
            entities.Remove(targetEntity);
        }
    }
}
