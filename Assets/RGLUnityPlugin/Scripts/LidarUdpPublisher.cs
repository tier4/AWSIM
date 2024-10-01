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

using System;
using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using UnityEngine.Assertions;

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

        public bool emitRawPackets = true;

        [Header("Hesai LiDARs Flags")]

        [Tooltip("Enable labeling the sequence number of Point Cloud UDP packets. It increases the packet size by an additional field.")]
        public bool enableHesaiUdpSequence = false;
        [Tooltip("Enable a workaround for the difference between the coordinate systems in the ROS2 driver and Hesai LiDAR manuals.")]
        public bool ensureHesaiRosDriverOrientation = false;  // When developing raw Hesai packets (based on LiDARs manuals),
                                                              // the difference between the coordinate systems in the ROS2 driver
                                                              // and LiDAR speciations was noted.
                                                              // Due to the use of the ROS driver on many real vehicles,
                                                              // it was decided to prepare a workaround in AWSIM.
        [Tooltip("Enable a feature that allows to distinguish point data between no laser emission and return signal rejection.")]
        public bool enableHesaiUpCloseBlockageDetection = false; // Only supported for Hesai QT128C2X

        [Tooltip("Hesai Pandar ROS2 driver has some differences from the LiDAR manuals. This flag applies these changes to raw packets.")]
        public bool ensureCompatibilityWithHesaiPandarDriver = false;

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
            { LidarModel.HesaiPandarQT, RGLLidarModel.RGL_HESAI_PANDAR_QT64 },
            { LidarModel.HesaiQT128C2X, RGLLidarModel.RGL_HESAI_QT128C2X },
            { LidarModel.HesaiPandar128E4X, RGLLidarModel.RGL_HESAI_PANDAR_128E4X },
            { LidarModel.HesaiPandarXT32, RGLLidarModel.RGL_HESAI_PANDAR_XT32 }
        };

        // Note: When selecting dual return mode, there will be still single return data but packed in the dual return packet format
        // The second return will be the same as the first return
        private static readonly Dictionary<LidarModel, List<RGLReturnMode>> SupportedLidarsAndReturnModes = new Dictionary<LidarModel, List<RGLReturnMode>>
        {
            { LidarModel.VelodyneVLP16, new List<RGLReturnMode>()
                { RGLReturnMode.SingleReturnStrongest, RGLReturnMode.SingleReturnLast, RGLReturnMode.DualReturnLastStrongest } },
            { LidarModel.VelodyneVLP32C, new List<RGLReturnMode>()
                { RGLReturnMode.SingleReturnStrongest, RGLReturnMode.SingleReturnLast, RGLReturnMode.DualReturnLastStrongest } },
            { LidarModel.VelodyneVLS128, new List<RGLReturnMode>()
                { RGLReturnMode.SingleReturnStrongest, RGLReturnMode.SingleReturnLast, RGLReturnMode.DualReturnLastStrongest } },
            { LidarModel.HesaiPandar40P, new List<RGLReturnMode>()
                { RGLReturnMode.SingleReturnStrongest, RGLReturnMode.SingleReturnLast, RGLReturnMode.DualReturnLastStrongest } },
            { LidarModel.HesaiPandarQT, new List<RGLReturnMode>()
                { RGLReturnMode.SingleReturnFirst, RGLReturnMode.SingleReturnLast, RGLReturnMode.DualReturnFirstLast } },
            { LidarModel.HesaiQT128C2X, new List<RGLReturnMode>()
                { RGLReturnMode.SingleReturnFirst, RGLReturnMode.SingleReturnSecond, RGLReturnMode.SingleReturnStrongest, RGLReturnMode.SingleReturnLast,
                  RGLReturnMode.DualReturnLastStrongest, RGLReturnMode.DualReturnFirstLast, RGLReturnMode.DualReturnFirstStrongest,
                  RGLReturnMode.DualReturnStrongestSecondStrongest, RGLReturnMode.DualReturnFirstSecond } },
            { LidarModel.HesaiPandar128E4X, new List<RGLReturnMode>()
                { RGLReturnMode.SingleReturnFirst, RGLReturnMode.SingleReturnStrongest, RGLReturnMode.SingleReturnLast,
                  RGLReturnMode.DualReturnLastStrongest, RGLReturnMode.DualReturnFirstLast, RGLReturnMode.DualReturnFirstStrongest } },
            { LidarModel.HesaiPandarXT32, new List<RGLReturnMode>()
                { RGLReturnMode.SingleReturnStrongest, RGLReturnMode.SingleReturnLast, RGLReturnMode.DualReturnLastStrongest } },
        };

        private bool IsVelodyne(LidarModel model)
        {
            return model == LidarModel.VelodyneVLP16 ||
                   model == LidarModel.VelodyneVLP32C ||
                   model == LidarModel.VelodyneVLS128;
        }

        private bool IsHesai(LidarModel model)
        {
            return model == LidarModel.HesaiPandar40P ||
                   model == LidarModel.HesaiPandarQT ||
                   model == LidarModel.HesaiPandarXT32 ||
                   model == LidarModel.HesaiQT128C2X ||
                   model == LidarModel.HesaiPandar128E4X;
        }

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

            if (!NetworkUtilities.IsValidIpAddress(sourceIPOnAwake))
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
            lidarSensor.onLidarModelChange += HandleNewLidarModel;

            // We can connect to world frame, because we need only DISTANCE field which is in lidar frame anyway.
            // This way we don't need to duplicate transform nodes to have compacted and non-compacted point cloud in lidar frame.
            lidarSensor.ConnectToWorldFrame(rglSubgraphUdpPublishing, false);

            HandleNewLidarModel();
        }

        public void OnValidate()
        {
            // `rglSubgraphUdpPublishing` is constructed on simulation startup. `OnValidate` works only in runtime.
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

            HandleNewLidarModel();
        }

        public void OnEnable()
        {
            rglSubgraphUdpPublishing?.SetActive(udpPublishingNodeId, emitRawPackets);
            HandleNewLidarModel();
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

        private void HandleNewLidarModel()
        {
            if (lidarSensor == null || !enabled)
            {
                return;
            }

            var modelToValidate = lidarSensor.configuration;
            // Horizontal field of view must be 360 degrees
            if (modelToValidate.maxHAngle - modelToValidate.minHAngle != 360)
            {
                Debug.LogError($"{name}: Lidar model configuration not supported for UDP publishing " +
                               "- horizontal field of view different than 360 degrees. Disabling component...");
                OnDisable();
            }

            LidarModel currentLidarModel = lidarSensor.modelPreset;

            if (!UnityToRGLLidarModelsMapping.ContainsKey(currentLidarModel))
            {
                Debug.LogError($"{name}: Lidar model preset not supported for UDP publishing. " +
                               $"Please select one of: [{string.Join(", ", UnityToRGLLidarModelsMapping.Keys)}]. Disabling component...");
                OnDisable();
                return;
            }
            if (!SupportedLidarsAndReturnModes.ContainsKey(currentLidarModel) || SupportedLidarsAndReturnModes[currentLidarModel].Count == 0)
            {
                Debug.LogError($"{name}: Lidar model preset doesn't have specification for supported return modes. It is most likely due to implementation error. " +
                               "Please contact to the project maintainers. Disabling component...");
                OnDisable();
                return;
            }

            // Check if lidar configuration doesn't exceed max range for Velodyne Legacy Packet Format
            // Currently, all of the supported Velodyne models use this packet format
            if (IsVelodyne(currentLidarModel) && modelToValidate.GetRayRanges().Max(v => v.y) > maxRangeForVelodyneLegacyPacketFormat)
            {
                Debug.LogWarning($"Max range of lidar '{lidarSensor.name}' exceeds max range supported by Velodyne Legacy Packet Format ({maxRangeForVelodyneLegacyPacketFormat}m). " +
                                 "Consider reducing its value to ensure proper work.");
            }

            // This is a workaround for the difference between the coordinate systems in the ROS2 driver and Hesai LiDAR manuals
            // The order of the points is changed to be encoded to match ROS2 coordinate system
            if (ensureHesaiRosDriverOrientation && IsHesai(currentLidarModel) && (lidarSensor.configuration.minHAngle != -90.0f || lidarSensor.configuration.maxHAngle != 270.0f))
            {
                lidarSensor.configuration.minHAngle = -90.0f;
                lidarSensor.configuration.maxHAngle = 270.0f;
                lidarSensor.OnValidate();  // This will trigger `HandleNewLidarModel()` again
                return;
            }

            // Check if supported return mode is selected
            if (!SupportedLidarsAndReturnModes[currentLidarModel].Contains(lidarSensor.returnMode))
            {
                Debug.LogError($"{name}: Return mode for selected lidar model preset is not supported. " +
                               $"Please select one of: [{string.Join(", ", SupportedLidarsAndReturnModes[currentLidarModel])}]. Disabling component...");
                OnDisable();
            }

            // Update RGL subgraph
            rglSubgraphUdpPublishing.UpdateNodePointsUdpPublish(
                udpPublishingNodeId, UnityToRGLLidarModelsMapping[currentLidarModel], GetUdpOptions(currentLidarModel),
                sourceIPOnAwake, destinationIPOnAwake, destinationPortOnAwake);
        }

        private RGLUdpOptions GetUdpOptions(LidarModel currentLidarModel)
        {
            // Validate current model and option flags
            if (!IsHesai(currentLidarModel) && enableHesaiUdpSequence)
            {
                enableHesaiUdpSequence = false;
                Debug.LogWarning($"{name}: enableHesaiUdpSequence option is not available for selected LiDAR model. Disabling option...");
            }
            // LiDARs that need udp sequence flag to be set
            if ((currentLidarModel == LidarModel.HesaiQT128C2X
                 || currentLidarModel == LidarModel.HesaiPandar128E4X
                 || currentLidarModel == LidarModel.HesaiPandarXT32)
                && !enableHesaiUdpSequence)
            {
                enableHesaiUdpSequence = true;
                Debug.LogWarning($"{name}: enableHesaiUdpSequence option must be enabled for selected LiDAR model. Enabling option...");
            }
            if (currentLidarModel != LidarModel.HesaiQT128C2X  && enableHesaiUpCloseBlockageDetection)
            {
                enableHesaiUpCloseBlockageDetection = false;
                Debug.LogWarning($"{name}: enableHesaiUpCloseBlockageDetection option is only available for Hesai QT128C2X LiDAR model. Disabling option...");
            }

            // Other LiDAR models are compatible with Hesai Pandar Driver by default
            bool enableHesaiPandarDriverCompatibilityForQt = currentLidarModel == LidarModel.HesaiPandarQT && ensureCompatibilityWithHesaiPandarDriver;

            // Construct RGLUdpOptions
            // We need to cast to the underlying type of the enum to be able to add multiple udp options
            Assert.IsTrue(Enum.GetUnderlyingType(typeof(RGLUdpOptions)) == typeof(UInt32)); // Check if we are casting properly
            UInt32 udpOptions = (UInt32)RGLUdpOptions.RGL_UDP_NO_ADDITIONAL_OPTIONS;
            udpOptions += enableHesaiUdpSequence ? (UInt32)RGLUdpOptions.RGL_UDP_ENABLE_HESAI_UDP_SEQUENCE : 0;
            udpOptions += enableHesaiUpCloseBlockageDetection ? (UInt32)RGLUdpOptions.RGL_UDP_UP_CLOSE_BLOCKAGE_DETECTION : 0;
            udpOptions += enableHesaiPandarDriverCompatibilityForQt ? (UInt32)RGLUdpOptions.RGL_UDP_FIT_QT64_TO_HESAI_PANDAR_DRIVER : 0;

            // Check if high resolution mode is enabled (available only on Hesai Pandar128E4X)
            if (currentLidarModel == LidarModel.HesaiPandar128E4X)
            {
                if (((HesaiPandar128E4XLidarConfiguration)lidarSensor.configuration).highResolutionModeEnabled)
                {
                    udpOptions += (UInt32)RGLUdpOptions.RGL_UDP_HIGH_RESOLUTION_MODE;
                }
            }

            return (RGLUdpOptions)udpOptions;
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

            Debug.LogError("Loaded RGL plugin does not include support for UDP Raw Packet, removing component");
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
