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

using ROS2;
using UnityEngine;
using Awsim.Common;

namespace Awsim.Usecase.AwsimRvizPlugins
{
    /// <summary>
    /// Inspector and calling each module of AwsimRvizPlugins
    /// </summary>
    public class AwsimRvizPluginsClient : MonoBehaviour
    {
        [Header("2D Pose Teleport")]
        [SerializeField] Transform _egoTransform;
        [SerializeField] string _egoPositionTopic = "/awsim/awsim_rviz_plugins/pose_teleport/pose_with_covariance";

        [Header("Npc Spawner")]
        [SerializeField] NpcSpawnerSettings _npcSpawnerSettings;

        [Header("ROS2")]
        [SerializeField]
        QosSettings _qosSettings = new QosSettings(ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
                                                   DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
                                                   HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
                                                   5);

        PoseTeleport _poseTeleport;
        NpcSpawner _npcSpawner;

        /// <summary>
        /// Initialize qos profile and module of AwsimRvizPlugins
        /// </summary>
        public void Initialize()
        {
            var qos = _qosSettings.GetQosProfile();

            _poseTeleport = new PoseTeleport(_egoTransform, _egoPositionTopic, qos);
            _npcSpawner = new NpcSpawner(this.transform, _npcSpawnerSettings, qos);
        }

        public void OnUpdate()
        {
            _npcSpawner.OnUpdate();
        }

        /// <summary>
        /// Fixed update module of AwsimRvizPlugins
        /// </summary>
        public void OnFixedUpdate()
        {
            _poseTeleport.OnFixedUpdate();
            _npcSpawner.OnFixedUpdate();
        }

        /// <summary>
        /// Destroy module of AwsimRvizPlugins
        /// </summary>
        void OnDestroy()
        {
            _poseTeleport.OnDestroy();
            _npcSpawner.OnDestroy();
        }
    }
}