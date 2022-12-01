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

using UnityEngine;
using AWSIM;
using AWSIM.PointCloudFormats;
using RGLUnityPlugin;

namespace AWSIM
{
    /// <summary>
    /// R2FU integration for Robotec GPU Lidar Unity Plugin.
    /// </summary>
    [RequireComponent(typeof(LidarSensor))]
    public class RglLidarPublisher : MonoBehaviour
    {
        public string pcl24Topic = "lidar/pointcloud";
        public string pcl48Topic = "lidar/pointcloud_ex";
        public string frameID = "world";
        public bool publishPCL24 = true;
        public bool publishPCL48 = true;

        public RGLQosPolicyReliability reliabilityPolicy = RGLQosPolicyReliability.QOS_POLICY_RELIABILITY_BEST_EFFORT;
        public RGLQosPolicyDurability durabilityPolicy = RGLQosPolicyDurability.QOS_POLICY_DURABILITY_VOLATILE;
        public RGLQosPolicyHistory historyPolicy = RGLQosPolicyHistory.QOS_POLICY_HISTORY_KEEP_LAST;
        public int historyDepth = 5;

        private RGLNodeSequence rglSubgraphUnity2Ros;
        private RGLNodeSequence rglSubgraphPcl24;
        private RGLNodeSequence rglSubgraphPcl48;

        private LidarSensor lidarSensor;

        private void Start()
        {
            if (!publishPCL24 && !publishPCL48)
            {
                Debug.LogWarning("All lidar message formats are disabled. Nothing to publish!");
            }

            lidarSensor = GetComponent<LidarSensor>();

            rglSubgraphUnity2Ros = new RGLNodeSequence()
                .AddNodePointsTransform("UNITY_TO_ROS", ROS2.Transformations.Unity2RosMatrix4x4());
            lidarSensor.ConnectToLidarFrame(rglSubgraphUnity2Ros);

            if (publishPCL24)
            {
                rglSubgraphPcl24 = new RGLNodeSequence()
                    .AddNodePointsFormat("PCL24", FormatPCL24.GetRGLFields())
                    .AddNodePointsRos2Publish("pub24", pcl24Topic, frameID, reliabilityPolicy, durabilityPolicy, historyPolicy, historyDepth);
                RGLNodeSequence.Connect(rglSubgraphUnity2Ros, rglSubgraphPcl24);
            }

            if (publishPCL48)
            {
                rglSubgraphPcl48 = new RGLNodeSequence()
                    .AddNodePointsFormat("PCL48", FormatPCL48.GetRGLFields())
                    .AddNodePointsRos2Publish("pub48", pcl48Topic, frameID, reliabilityPolicy, durabilityPolicy, historyPolicy, historyDepth);
                RGLNodeSequence.Connect(rglSubgraphUnity2Ros, rglSubgraphPcl48);
            }
        }

        public void OnValidate()
        {
            if (rglSubgraphPcl24 != null)
            {
                if (publishPCL24 != rglSubgraphPcl24.IsActive())
                {
                    rglSubgraphPcl24.SetActive(publishPCL24);
                }
            }

            if (rglSubgraphPcl48 != null)
            {
                if (publishPCL48 != rglSubgraphPcl48.IsActive())
                {
                    rglSubgraphPcl48.SetActive(publishPCL48);
                }
            }
        }
    }
}