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
using ROS2;
using UnityEngine.Profiling;

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

        public QoSSettings qosSettings = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 5,
        };

        private Publisher<sensor_msgs.msg.PointCloud2> pcl24Publisher;
        private Publisher<sensor_msgs.msg.PointCloud2> pcl48Publisher;
        private sensor_msgs.msg.PointCloud2 pcl24SensorMsg;
        private sensor_msgs.msg.PointCloud2 pcl48SensorMsg;

        private RGLNodeSequence rglSubgraphUnity2Ros;
        private RGLNodeSequence rglSubgraphPcl24;
        private RGLNodeSequence rglSubgraphPcl48;

        private byte[] pcl24Data;
        private byte[] pcl48Data;
        private LidarSensor lidarSensor;

        private void Start()
        {
            if (!publishPCL24 && !publishPCL48)
            {
                Debug.LogWarning("All lidar message formats are disabled. Nothing to publish!");
            }

            lidarSensor = GetComponent<LidarSensor>();
            lidarSensor.onNewData += OnNewLidarData;

            rglSubgraphUnity2Ros = new RGLNodeSequence()
                .AddNodePointsTransform("UNITY_TO_ROS", ROS2.Transformations.Unity2RosMatrix4x4());
            lidarSensor.ConnectToLidarFrame(rglSubgraphUnity2Ros);

            if (publishPCL24)
            {
                pcl24Data = new byte[0];
                pcl24Publisher = SimulatorROS2Node.CreatePublisher<sensor_msgs.msg.PointCloud2>(pcl24Topic, qosSettings.GetQoSProfile());
                pcl24SensorMsg = FormatPCL24.GetSensorMsg();
                pcl24SensorMsg.SetHeaderFrame(frameID);
                rglSubgraphPcl24 = new RGLNodeSequence()
                    .AddNodePointsFormat("PCL24", FormatPCL24.GetRGLFields());
                RGLNodeSequence.Connect(rglSubgraphUnity2Ros, rglSubgraphPcl24);
            }

            if (publishPCL48)
            {
                pcl48Data = new byte[0];
                pcl48Publisher = SimulatorROS2Node.CreatePublisher<sensor_msgs.msg.PointCloud2>(pcl48Topic, qosSettings.GetQoSProfile());
                pcl48SensorMsg = FormatPCL48.GetSensorMsg();
                pcl48SensorMsg.SetHeaderFrame(frameID);
                rglSubgraphPcl48 = new RGLNodeSequence()
                    .AddNodePointsFormat("PCL48", FormatPCL48.GetRGLFields());
                RGLNodeSequence.Connect(rglSubgraphUnity2Ros, rglSubgraphPcl48);
            }
        }

        private void OnNewLidarData()
        {
            Profiler.BeginSample("Publish Pointclouds");
            if (publishPCL24)
            {
                int hitCount = rglSubgraphPcl24.GetResultDataRaw(ref pcl24Data, 24);
                PublishFormat(pcl24Publisher, pcl24SensorMsg, pcl24Data, hitCount);
            }

            if (publishPCL48)
            {
                int hitCount = rglSubgraphPcl48.GetResultDataRaw(ref pcl48Data, 48);
                PublishFormat(pcl48Publisher, pcl48SensorMsg, pcl48Data, hitCount);
            }
            Profiler.EndSample();
        }

        private void PublishFormat(Publisher<sensor_msgs.msg.PointCloud2> publisher, sensor_msgs.msg.PointCloud2 msg,
            byte[] data, int hitCount)
        {
            var header = msg as MessageWithHeader;
            SimulatorROS2Node.UpdateROSTimestamp(ref header);
            msg.Data = data;
            msg.Width = (uint) hitCount;
            msg.Row_step = msg.Point_step * msg.Width;
            publisher.Publish(msg);
        }

        private void OnDisable()
        {
            if(pcl24Publisher != null) pcl24Publisher.Dispose();
            if(pcl48Publisher != null) pcl48Publisher.Dispose();
        }
    }
}