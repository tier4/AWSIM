using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;

namespace AWSIM
{
    public class Ros2EntityController : MonoBehaviour
    {
        [Serializable]
        public class NPCVehiclePrefab
        {
            public string AssetKey;
            public NPCVehicle prefab;
        }

        [Serializable]
        public class NPCPedestrianPrefab
        {
            public string AssetKey;
            public NPCPedestrian prefab;
        }
        
        [Serializable]
        public class TransformPrefab
        {
            public string AssetKey;
            public Transform prefab;
        }

        [Serializable]
        public class Ros2Config
        {
            public string Topic;
            public QoSSettings QoS;
        }

        [Header("Entity")]
        public NPCVehiclePrefab[] npcVehiclePrefabs;
        public NPCPedestrianPrefab[] npcPedestrianPrefabs;
        public TransformPrefab[] transformPrefabs;

        [Header("ROS2 config")]
        public Ros2Config Spawn = new Ros2Config()
        {
            Topic = "awsim/ros2-entity-control/spawn",
            QoS = new QoSSettings()
            {
                ReliabilityPolicy = ROS2.ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
                DurabilityPolicy = ROS2.DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
                HistoryPolicy = ROS2.HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
                Depth = 1,
            }
        };
        public Ros2Config UpdatePose = new Ros2Config()
        {
            Topic = "awsim/ros2-entity-control/desspawn",
            QoS = new QoSSettings()
            {
                ReliabilityPolicy = ROS2.ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT,
                DurabilityPolicy = ROS2.DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
                HistoryPolicy = ROS2.HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
                Depth = 5,
            }
        };
        public Ros2Config Despawn = new Ros2Config()
        {
            Topic = "awsim/ros2-entity-control/desspawn",
            QoS = new QoSSettings()
            {
                ReliabilityPolicy = ROS2.ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
                DurabilityPolicy = ROS2.DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
                HistoryPolicy = ROS2.HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
                Depth = 1,
            }
        };

        void Start()
        {
            // Prevent duplicate asset keys
            var npcVehicleKeys = npcVehiclePrefabs.Select(x => x.AssetKey);
            var npcPedestrianKeys = npcPedestrianPrefabs.Select(x => x.AssetKey);
            var transformKeys = transformPrefabs.Select(x => x.AssetKey);
            var allkeys = npcVehicleKeys.Concat(npcPedestrianKeys).Concat(transformKeys);
            bool duplicatedAssetKey = allkeys.GroupBy(x => x).Where(x => x.Count() > 1).Count() > 0;
            if (duplicatedAssetKey)
            {
                Debug.LogError("Duplicate asset key");
                return;
            }

            // TODO: create spawn ROS2 subscriber.

            // TODO: create update pose ROS2 subscriber.

            // TODO: create despawn ROS2 subscriber.
        }
    }
}