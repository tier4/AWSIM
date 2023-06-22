// Copyright 2023 Robotec.ai.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using UnityEngine;
using System.Linq;

namespace RGLUnityPlugin
{
    [RequireComponent(typeof(LidarSensor))]
    public class LidarUdpPublisher : MonoBehaviour
    {
        public string sourceIP = "0.0.0.0";
        public string destinationIP = "255.255.255.255";
        public int destinationPort = 2368;
        public bool emitRawPackets = true;

        private RGLNodeSequence rglSubgraphUdpPublishing;

        private readonly string udpPublishingNodeId = "UDP_PUBLISHING";

        private LidarSensor lidarSensor;

        // To be called when adding this component
        private void Reset()
        {
            CheckAndDestroyComponentIfUnsupported();
        }

        private void Awake()
        {
            if (CheckAndDestroyComponentIfUnsupported())
            {
                return;
            }

            rglSubgraphUdpPublishing = new RGLNodeSequence()
                .AddNodePointsUdpPublishVlp16(udpPublishingNodeId, sourceIP, destinationIP, destinationPort);
        }

        private void Start()
        {
            lidarSensor = GetComponent<LidarSensor>();
            lidarSensor.onLidarModelChange += ValidateLidarModel;

            // We can connect to world frame, because we need only DISTANCE field which is in lidar frame anyway.
            // This way we don't need to dupicate transform nodes to have compacted and non-compacted point cloud in lidar frame.
            lidarSensor.ConnectToWorldFrame(rglSubgraphUdpPublishing, false);

            ValidateLidarModel();
        }

        public void OnValidate()
        {
            if (rglSubgraphUdpPublishing == null)
            {
                return;
            }

            if (emitRawPackets != rglSubgraphUdpPublishing.IsActive(udpPublishingNodeId))
            {
                rglSubgraphUdpPublishing.SetActive(udpPublishingNodeId, emitRawPackets);
            }
        }

        public void OnEnable()
        {
            rglSubgraphUdpPublishing?.SetActive(udpPublishingNodeId, emitRawPackets);
            ValidateLidarModel();
        }

        public void OnDisable()
        {
            rglSubgraphUdpPublishing?.SetActive(udpPublishingNodeId, false);
            enabled = false;
        }

        public void OnDestroy()
        {
            rglSubgraphUdpPublishing?.Clear();
            rglSubgraphUdpPublishing = null;
        }

        private void ValidateLidarModel()
        {
            if (lidarSensor == null || !enabled)
            {
                return;
            }

            var modelToValidate = lidarSensor.configuration;
            if (modelToValidate.maxHAngle - modelToValidate.minHAngle != 360)
            {
                Debug.LogError("Lidar model configuration not supported for UDP publishing " +
                               "- horizontal field of view different than 360 degrees. Disabling component...");
                OnDisable();
            }

            var vlp16Model = LidarConfigurationLibrary.ByModel[LidarModel.VelodyneVLP16];
            if (!modelToValidate.laserArray.lasers.SequenceEqual(vlp16Model.laserArray.lasers))
            {
                Debug.LogError("Lidar model configuration not supported for UDP publishing " +
                               "- lasers doesn't match Velodyne VLP16. Disabling component...");
                OnDisable();
            }
        }

        /// <summary>
        /// Returns true if component has been destroyed
        /// </summary>
        private bool CheckAndDestroyComponentIfUnsupported()
        {
            if (RGLNativeAPI.HasExtension(RGLExtension.RGL_EXTENSION_UDP))
            {
                return false;
            }

            Debug.LogError($"Loaded RGL plugin does not include support for UDP Raw Packet, removing component");
            if (Application.isEditor && !Application.isPlaying)  // In edit mode
            {
                DestroyImmediate(this);
            }
            else
            {
                Destroy(this);
            }
            return true;
        }
    }
}
