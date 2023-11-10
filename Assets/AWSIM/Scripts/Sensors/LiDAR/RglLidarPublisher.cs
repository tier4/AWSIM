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
using AWSIM.PointCloudFormats;
using RGLUnityPlugin;

namespace AWSIM
{
    /// <summary>
    /// ROS2 publishing component for Robotec GPU Lidar Unity Plugin.
    /// </summary>
    [RequireComponent(typeof(LidarSensor))]
    public class RglLidarPublisher : MonoBehaviour
    {
        public string pcl24Topic = "lidar/pointcloud";
        public string pcl48Topic = "lidar/pointcloud_ex";
        public string instanceIdTopic = "lidar/instance_id";
        public string frameID = "world";
        public bool publishPCL24 = true;
        public bool publishPCL48 = true;
        public bool publishInstanceId = false;

        public RGLQosPolicyReliability reliabilityPolicy = RGLQosPolicyReliability.QOS_POLICY_RELIABILITY_BEST_EFFORT;
        public RGLQosPolicyDurability durabilityPolicy = RGLQosPolicyDurability.QOS_POLICY_DURABILITY_VOLATILE;
        public RGLQosPolicyHistory historyPolicy = RGLQosPolicyHistory.QOS_POLICY_HISTORY_KEEP_LAST;
        public int historyDepth = 5;

        private RGLNodeSequence rglSubgraphUnity2Ros;
        private RGLNodeSequence rglSubgraphPcl24;
        private RGLNodeSequence rglSubgraphPcl48;
        private RGLNodeSequence rglSubgraphInstanceId;

        private LidarSensor lidarSensor;

        private bool didStart = false;

        private void Awake()
        {
            rglSubgraphUnity2Ros = new RGLNodeSequence()
                .AddNodePointsTransform("UNITY_TO_ROS", ROS2.Transformations.Unity2RosMatrix4x4());
        }

        private void OnEnable()
        {
            if (publishPCL24 && rglSubgraphPcl24 == null)
            {
                rglSubgraphPcl24 = new RGLNodeSequence()
                    .AddNodePointsFormat("PCL24_FORMAT", FormatPCL24.GetRGLFields())
                    .AddNodePointsRos2Publish("PCL24_PUB", pcl24Topic, frameID, reliabilityPolicy, durabilityPolicy, historyPolicy, historyDepth);
                RGLNodeSequence.Connect(rglSubgraphUnity2Ros, rglSubgraphPcl24);
            }

            if (publishPCL48 && rglSubgraphPcl48 == null)
            {
                rglSubgraphPcl48 = new RGLNodeSequence()
                    .AddNodePointsFormat("PCL48_FORMAT", FormatPCL48.GetRGLFields())
                    .AddNodePointsRos2Publish("PCL48_PUB", pcl48Topic, frameID, reliabilityPolicy, durabilityPolicy, historyPolicy, historyDepth);
                RGLNodeSequence.Connect(rglSubgraphUnity2Ros, rglSubgraphPcl48);
            }

            if (publishInstanceId && rglSubgraphInstanceId == null)
            {
                rglSubgraphInstanceId = new RGLNodeSequence()
                    .AddNodePointsFormat("ML_FORMAT", FormatMLInstanceSegmentation.GetRGLFields())
                    .AddNodePointsRos2Publish("ML_PUB", instanceIdTopic, frameID, reliabilityPolicy, durabilityPolicy, historyPolicy, historyDepth);
                RGLNodeSequence.Connect(rglSubgraphUnity2Ros, rglSubgraphInstanceId);
            }
        }

        private void Start()
        {
            if (!publishPCL24 && !publishPCL48 && !publishInstanceId)
            {
                Debug.LogWarning("All lidar message formats are disabled. Nothing to publish!");
            }

            // Synchronize RGL time with the same TimeSource as AWSIM
            SceneManager.TimeSource = SimulatorROS2Node.TimeSource;

            lidarSensor = GetComponent<LidarSensor>();
            lidarSensor.ConnectToLidarFrame(rglSubgraphUnity2Ros);

            didStart = true;
        }

        public void OnValidate()
        {
            if (!enabled)
            {
                return;
            }

            ApplySubgraphState(ref rglSubgraphPcl24, publishPCL24);
            ApplySubgraphState(ref rglSubgraphPcl48, publishPCL48);
            ApplySubgraphState(ref rglSubgraphInstanceId, publishInstanceId);
        }

        private void OnDisable()
        {
            ApplySubgraphState(ref rglSubgraphPcl24, false);
            ApplySubgraphState(ref rglSubgraphPcl48, false);
            ApplySubgraphState(ref rglSubgraphInstanceId, false);
        }

        private void OnDestroy()
        {
            Action<RGLNodeSequence> destroySubgraphIfNotNull = subgraph =>
            {
                if (rglSubgraphPcl24 == null) return;
                rglSubgraphPcl24.Clear();
                rglSubgraphPcl24 = null;
            };

            destroySubgraphIfNotNull(rglSubgraphPcl24);
            destroySubgraphIfNotNull(rglSubgraphPcl48);
            destroySubgraphIfNotNull(rglSubgraphInstanceId);
            destroySubgraphIfNotNull(rglSubgraphUnity2Ros);
        }

        private void ApplySubgraphState(ref RGLNodeSequence subgraph, bool activateState)
        {
            if (subgraph == null)
            {
                if (didStart && activateState == true)
                {
                    Debug.LogWarning("Cannot activate the publisher because it was not created. Please activate it before the simulation starts.");
                }
                return;
            }

            if (activateState == RGLNodeSequence.AreConnected(rglSubgraphUnity2Ros, subgraph))
            {
                return;
            }

            if (activateState)
            {
                RGLNodeSequence.Connect(rglSubgraphUnity2Ros, subgraph);
            }
            else
            {
                RGLNodeSequence.Disconnect(rglSubgraphUnity2Ros, subgraph);
            }
        }
    }
}
