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
using System.Collections.Generic;
using System.Linq;
using System.Net.NetworkInformation;

namespace RGLUnityPlugin
{
    [RequireComponent(typeof(LidarSensor))]
    public class LidarUdpPublisher : MonoBehaviour
    {
        public string sourceIP = "0.0.0.0";
        public string destinationIP = "255.255.255.255";
        public int destinationPort = 2368;

        private string sourceIPOnAwake;
        private string destinationIPOnAwake;
        private int destinationPortOnAwake;

        public bool enableHesaiUdpSequence = false;
        public bool useDualReturnFormat = false;  // Still single return data but packed in the dual return packet format
                                                  // The second return will be the same as the first return
        public bool ensureHesaiRosDriverOrientation = false;  // When developing raw Hesai packets (based on LiDARs manuals),
                                                              // the difference between the coordinate systems in the ROS2 driver
                                                              // and LiDAR speciations was noted.
                                                              // Due to the use of the ROS driver on many real vehicles,
                                                              // it was decided to prepare a workaround in AWSIM.
        public bool emitRawPackets = true;

        private RGLLidarModel currentRGLLidarModel = 0;  // To be set when validating lidar model
        private RGLUdpOptions currentRGLUdpOptions = 0;  // To be set when validating lidar model
        private RGLNodeSequence rglSubgraphUdpPublishing;

        private const string udpPublishingNodeId = "UDP_PUBLISHING";

        private LidarSensor lidarSensor;

        private static readonly float maxRangeForVelodyneLegacyPacketFormat = 262.14f;

        private static readonly Dictionary<LidarModel, RGLLidarModel> UnityToRGLLidarModelsMapping = new Dictionary<LidarModel, RGLLidarModel>
        {
            { LidarModel.VelodyneVLP16, RGLLidarModel.RGL_VELODYNE_VLP16 },
            { LidarModel.VelodyneVLP32C, RGLLidarModel.RGL_VELODYNE_VLP32C },
            { LidarModel.VelodyneVLS128, RGLLidarModel.RGL_VELODYNE_VLS128 },
            { LidarModel.HesaiPandar40P, RGLLidarModel.RGL_HESAI_PANDAR_40P },
            { LidarModel.HesaiPandarQT, RGLLidarModel.RGL_HESAI_PANDAR_QT64 }
        };

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

            sourceIPOnAwake = sourceIP;
            destinationIPOnAwake = destinationIP;
            destinationPortOnAwake = destinationPort;

            if (!IsValidIpAddress(sourceIPOnAwake))
            {
                var detailedName = transform.parent != null ? $"{transform.parent.name}:{name}" : name;
                Debug.LogError($"{detailedName}: IP address '{sourceIPOnAwake}' is invalid. " +
                               "Disabling component. Please restart the simulation with a correct IP address.");
                OnDisable();
                return;
            }

            // Node parameters will be updated when validating lidar model
            rglSubgraphUdpPublishing = new RGLNodeSequence()
                .AddNodePointsUdpPublish(udpPublishingNodeId, RGLLidarModel.RGL_VELODYNE_VLP16, RGLUdpOptions.RGL_UDP_NO_ADDITIONAL_OPTIONS,
                    sourceIPOnAwake, destinationIPOnAwake, destinationPortOnAwake);
        }

        private void Start()
        {
            lidarSensor = GetComponent<LidarSensor>();
            lidarSensor.onLidarModelChange += ValidateLidarModel;

            // We can connect to world frame, because we need only DISTANCE field which is in lidar frame anyway.
            // This way we don't need to duplicate transform nodes to have compacted and non-compacted point cloud in lidar frame.
            lidarSensor.ConnectToWorldFrame(rglSubgraphUdpPublishing, false);

            if (ensureHesaiRosDriverOrientation)
            {
                OnValidate();  // Needed to handle this flag on startup
            }

            ValidateLidarModel();
        }

        public void OnValidate()
        {
            if (rglSubgraphUdpPublishing == null)
            {
                return;
            }

            if (sourceIPOnAwake != sourceIP)
            {
                sourceIP = sourceIPOnAwake;
                Debug.LogWarning("`sourceIP` parameter cannot be changed in simulation runtime");
            }

            if (destinationIPOnAwake != destinationIP)
            {
                destinationIP = destinationIPOnAwake;
                Debug.LogWarning("`destinationIP` parameter cannot be changed in simulation runtime");
            }

            if (destinationPortOnAwake != destinationPort)
            {
                destinationPort = destinationPortOnAwake;
                Debug.LogWarning("`destinationPort` parameter cannot be changed in simulation runtime");
            }

            if (emitRawPackets != rglSubgraphUdpPublishing.IsActive(udpPublishingNodeId))
            {
                rglSubgraphUdpPublishing.SetActive(udpPublishingNodeId, emitRawPackets);
            }

            if (ensureHesaiRosDriverOrientation && (lidarSensor.configuration.minHAngle != -90.0f || lidarSensor.configuration.maxHAngle != 270.0f))
            {
                lidarSensor.configuration.minHAngle = -90.0f;
                lidarSensor.configuration.maxHAngle = 270.0f;
                lidarSensor.OnValidate();
                return;  // UpdateRGLSubgraph() will be called when validating new LiDAR model configuration
            }

            UpdateRGLSubgraph();
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

            LidarModel? detectedUnityLidarModel = null;
            foreach(var unityLidarModel in UnityToRGLLidarModelsMapping.Keys)
            {
                var unityLidarConfig = LidarConfigurationLibrary.ByModel[unityLidarModel];
                if (modelToValidate.laserArray.lasers.SequenceEqual(unityLidarConfig.laserArray.lasers))
                {
                    detectedUnityLidarModel = unityLidarModel;
                    break;
                }
            }

            if (detectedUnityLidarModel == null)
            {
                Debug.LogError("Lidar model configuration not supported for UDP publishing " +
                               $"- lasers doesn't match any supported Lidar models ({string.Join(", ", UnityToRGLLidarModelsMapping.Keys)}). Disabling component...");
                OnDisable();
                return;
            }

            // Currently, all of the supported Velodyne models use Legacy Packet Format
            if (IsVelodyne((LidarModel)detectedUnityLidarModel) && modelToValidate.maxRange > maxRangeForVelodyneLegacyPacketFormat)
            {
                Debug.LogWarning($"Max range of lidar '{lidarSensor.name}' exceeds max range supported by Velodyne Legacy Packet Format (262.14m). Consider reducing its value to ensure proper work.");
            }

            currentRGLLidarModel = UnityToRGLLidarModelsMapping[detectedUnityLidarModel.Value];
            UpdateRGLSubgraph();
        }

        private void UpdateRGLSubgraph()
        {
            HandleUdpOptionsFlags();
            rglSubgraphUdpPublishing.UpdateNodePointsUdpPublish(
                udpPublishingNodeId, currentRGLLidarModel, currentRGLUdpOptions,
                sourceIPOnAwake, destinationIPOnAwake, destinationPortOnAwake);
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

        private void HandleUdpOptionsFlags()
        {
            bool currentLidarIsHesai = currentRGLLidarModel == RGLLidarModel.RGL_HESAI_PANDAR_40P ||
                                       currentRGLLidarModel == RGLLidarModel.RGL_HESAI_PANDAR_QT64;

            if (!currentLidarIsHesai)
            {
                if (enableHesaiUdpSequence)
                {
                    enableHesaiUdpSequence = false;
                    currentRGLUdpOptions = RGLUdpOptions.RGL_UDP_NO_ADDITIONAL_OPTIONS;
                    Debug.LogWarning($"{name}: enableHesaiUdpSequence option is not available for selected LiDAR model. Disabling...");
                }

                if (useDualReturnFormat)
                {
                    useDualReturnFormat = false;
                    currentRGLUdpOptions = RGLUdpOptions.RGL_UDP_NO_ADDITIONAL_OPTIONS;
                    Debug.LogWarning($"{name}: useDualReturnFormat option is not available for selected LiDAR model. Disabling...");
                }

                return;
            }

            int udpOptionsConstruction = (int)RGLUdpOptions.RGL_UDP_NO_ADDITIONAL_OPTIONS;
            udpOptionsConstruction += enableHesaiUdpSequence ? (int)RGLUdpOptions.RGL_UDP_ENABLE_HESAI_UDP_SEQUENCE : 0;
            udpOptionsConstruction += useDualReturnFormat ? (int)RGLUdpOptions.RGL_UDP_DUAL_RETURN : 0;

            currentRGLUdpOptions = (RGLUdpOptions)udpOptionsConstruction;
        }

        private bool IsVelodyne(LidarModel model)
        {
            return model == LidarModel.VelodyneVLP16 ||
                   model == LidarModel.VelodyneVLP32C ||
                   model == LidarModel.VelodyneVLS128;
        }

        private bool IsValidIpAddress(in string ip)
        {
            if (ip == "0.0.0.0") return true;
            foreach (var nic in NetworkInterface.GetAllNetworkInterfaces())
            {
                foreach (var unicast in nic.GetIPProperties().UnicastAddresses)
                {
                    if (unicast.Address.ToString() == ip) return true;
                }
            }
            return false;
        }
    }
}
