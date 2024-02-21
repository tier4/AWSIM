// Copyright 2022 Robotec.ai.
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
using RGLUnityPlugin;

namespace AWSIM
{
    /// <summary>
    /// Quality of service settings for RGL
    /// </summary>
    [Serializable]
    public class RglQos
    {
        public RGLQosPolicyReliability reliabilityPolicy = RGLQosPolicyReliability.QOS_POLICY_RELIABILITY_BEST_EFFORT;
        public RGLQosPolicyDurability durabilityPolicy = RGLQosPolicyDurability.QOS_POLICY_DURABILITY_VOLATILE;
        public RGLQosPolicyHistory historyPolicy = RGLQosPolicyHistory.QOS_POLICY_HISTORY_KEEP_LAST;
        public int historyDepth = 5;
    }

    /// <summary>
    /// Base class for publisher. It contains:
    /// - serializable fields for topic information and checkbox for publishing activation
    /// - RGL subgraph
    /// - Abstract methods for initialization (creating subgraph) and validation (handling parameters update)
    /// - Method for instant subgraph destruction
    /// </summary>
    [Serializable]
    public abstract class BasePublisher
    {
        [Tooltip("Topic name to publish on. Resolved on simulation startup only.")]
        public string topic = "topic";
        [Tooltip("If false, publishing will be stopped.")]
        public bool publish = true;

        protected RGLNodeSequence publisherSubgraph;

        public void OnDestroy()
        {
            publisherSubgraph?.Clear();
        }

        public abstract void OnValidate();
        public abstract void Initialize(RGLNodeSequence parentSubgraph, string frameId, RglQos qos);
    }

    /// <summary>
    /// Publisher for PointCloud2 message
    /// </summary>
    [Serializable]
    public class PointCloud2Publisher : BasePublisher
    {
        private const string FormatNodeId = "FORMAT";
        private const string PublishNodeId = "PUBLISH";

        [Tooltip("Allows to select one of the built-in PointCloud2 fieldsets.")]
        public PointCloudFormat fieldsPreset;
        private PointCloudFormat? fieldsPresetPrev = null;
        [Tooltip("Fields to be present in the message.")]
        public RGLField[] fields;

        public override void OnValidate()
        {
            // If fields preset has changed, update fields array
            if (fieldsPresetPrev == null || fieldsPreset != fieldsPresetPrev)
            {
                // Do not update fields if preset is Custom (it may override user selection)
                if (fieldsPreset != PointCloudFormat.Custom)
                {
                    fields = PointCloudFormatLibrary.ByFormat[fieldsPreset];
                }
                fieldsPresetPrev = fieldsPreset;
            }

            // To enable/disable subgraph apply desired state to both nodes
            publisherSubgraph?.SetActive(FormatNodeId, publish);
            publisherSubgraph?.SetActive(PublishNodeId, publish);
        }

        public override void Initialize(RGLNodeSequence parentSubgraph, string frameId, RglQos qos)
        {
            publisherSubgraph = new RGLNodeSequence()
                .AddNodePointsFormat(FormatNodeId, fields)
                .AddNodePointsRos2Publish(PublishNodeId, topic, frameId, qos.reliabilityPolicy, qos.durabilityPolicy, qos.historyPolicy, qos.historyDepth);
            RGLNodeSequence.Connect(parentSubgraph, publisherSubgraph);
        }
    }

    /// <summary>
    /// Publisher for RadarScan message
    /// </summary>
    [Serializable]
    public class RadarScanPublisher : BasePublisher
    {
        private const string PublishNodeId = "PUBLISH";

        public override void OnValidate()
        {
            publisherSubgraph?.SetActive(PublishNodeId, publish);
        }

        public override void Initialize(RGLNodeSequence parentSubgraph, string frameId, RglQos qos)
        {
            publisherSubgraph = new RGLNodeSequence()
                .AddNodePublishRos2RadarScan(PublishNodeId, topic, frameId, qos.reliabilityPolicy, qos.durabilityPolicy, qos.historyPolicy, qos.historyDepth);
            RGLNodeSequence.Connect(parentSubgraph, publisherSubgraph);
        }
    }

    public enum PublisherType : UInt32
    {
        PointCloud2,
        RadarScan
    }

    /// <summary>
    /// Wrapper for the publisher that changes its type depending on the PublisherType enum.
    /// </summary>
    [Serializable]
    public class PublisherWrapper
    {
        public PublisherType publisherType;
        private PublisherType? publisherTypePrev = null;
        [SerializeReference]
        public BasePublisher publisher;

        private bool isInitialized = false;

        // Allow creating PublisherWrapper based on any BasePublisher
        public PublisherWrapper(BasePublisher publisher)
        {
            publisherType = publisher switch
            {
                PointCloud2Publisher _ => PublisherType.PointCloud2,
                RadarScanPublisher _ => PublisherType.RadarScan,
                _ => throw new ArgumentOutOfRangeException()
            };
            publisherTypePrev = publisherType;
            this.publisher = publisher;
        }

        // Implicit conversion from any BasePublisher to PublisherWrapper
        public static implicit operator PublisherWrapper(BasePublisher publisher)
        {
            return new PublisherWrapper(publisher);
        }

        public void OnValidate()
        {
            // If publisher type has changed, create a new publisher
            if (publisherTypePrev == null || publisherType != publisherTypePrev)
            {
                publisher = publisherType switch
                {
                    PublisherType.PointCloud2 => new PointCloud2Publisher(),
                    PublisherType.RadarScan => new RadarScanPublisher(),
                    _ => throw new ArgumentOutOfRangeException()
                };
                publisherTypePrev = publisherType;
            }
            publisher.OnValidate();
        }

        public void OnDestroy()
        {
            publisher.OnDestroy();
        }

        public void Initialize(RGLNodeSequence parentSubgraph, string frameId, RglQos qos)
        {
            if (isInitialized)
            {
                throw new InvalidOperationException("Publisher is already initialized");
            }
            publisher.Initialize(parentSubgraph, frameId, qos);
            isInitialized = true;
        }
    }

    /// <summary>
    /// ROS2 publishing component for Robotec GPU Lidar Unity Plugin.
    /// </summary>
    public class RglLidarPublisher : MonoBehaviour
    {
        [Tooltip("Frame the data is associated with. Resolved on simulation startup only.")]
        public string frameId = "world";
        [Tooltip("Quality of service settings. Resolved on simulation startup only.")]
        public RglQos qos;

        [Tooltip(
            "Array of publishers. They are initialized on simulation startup only and updates of parameters are not supported.")]
        public PublisherWrapper[] publishers =
        {
            new PointCloud2Publisher()
            {
                topic = "lidar/pointcloud",
                publish = true,
                fieldsPreset = PointCloudFormat.Pcl24,
            },
            new PointCloud2Publisher()
            {
                topic = "lidar/pointcloud_ex",
                publish = true,
                fieldsPreset = PointCloudFormat.Pcl48,
            },
        };

        private const string TransformNodeId = "UNITY_TO_ROS";

        private RGLNodeSequence rglSubgraphUnity2Ros;

        private void Awake()
        {
            rglSubgraphUnity2Ros = new RGLNodeSequence()
                .AddNodePointsTransform(TransformNodeId, ROS2.Transformations.Unity2RosMatrix4x4());
        }

        private void Start()
        {
            MonoBehaviour sensor = null;
            // Check if LiDAR is attached
            var lidar = GetComponent<LidarSensor>();
            if (lidar != null)
            {
                lidar.ConnectToLidarFrame(rglSubgraphUnity2Ros);
                sensor = lidar;
            }

            // Check if radar is attached
            var radar = GetComponent<RadarSensor>();
            if (radar != null)
            {
                if (sensor != null)
                {
                    Debug.LogError($"More than one sensor is attached to the publisher. Destroying {name}.");
                    Destroy(this);
                    return;
                }
                radar.ConnectToRadarFrame(rglSubgraphUnity2Ros);
                sensor = radar;
            }

            if (sensor == null)
            {
                Debug.LogError($"Cannot publish point cloud to ROS2 without sensor. Destroying {name}.");
                Destroy(this);
                return;
            }

            foreach (var publisher in publishers)
            {
                publisher.Initialize(rglSubgraphUnity2Ros, frameId, qos);
            }
        }

        public void OnValidate()
        {
            foreach (var publisher in publishers)
            {
                publisher.OnValidate();
            }
        }

        private void OnEnable()
        {
            rglSubgraphUnity2Ros?.SetActive(TransformNodeId, true);
        }

        private void OnDisable()
        {
            // Only one node in this subgraph so the whole subgraph will be disconnected from the sensor graph
            rglSubgraphUnity2Ros?.SetActive(TransformNodeId, false);
        }

        private void OnDestroy()
        {
            foreach (var publisher in publishers)
            {
                publisher.OnDestroy();
            }
            rglSubgraphUnity2Ros?.Clear();
        }
    }
}
