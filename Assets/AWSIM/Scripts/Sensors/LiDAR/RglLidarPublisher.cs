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
using rcl_interfaces.msg;
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
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 1000,
        };

        private Publisher<sensor_msgs.msg.PointCloud2> pcl24Publisher;
        private Publisher<sensor_msgs.msg.PointCloud2> pcl48Publisher;
        private sensor_msgs.msg.PointCloud2 pcl24SensorMsg;
        private sensor_msgs.msg.PointCloud2 pcl48SensorMsg;
        private RGLNodeHandle transformUnity2RosHandle;
        private RGLOutputHandle pcl24RglHandle;
        private RGLOutputHandle pcl48RglHandle;
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

            transformUnity2RosHandle = lidarSensor.AddPointsTransform(lidarSensor.GetPointsLidarFrameNodeHandle(), ROS2.Transformations.Unity2RosMatrix4x4());

            if (publishPCL24)
            {
                pcl24Data = new byte[0];
                pcl24Publisher = SimulatorROS2Node.CreatePublisher<sensor_msgs.msg.PointCloud2>(pcl24Topic, qosSettings.GetQoSProfile());
                pcl24RglHandle = lidarSensor.AddFormat(transformUnity2RosHandle, FormatPCL24.GetRGLFields());
                pcl24SensorMsg = FormatPCL24.GetSensorMsg();
                pcl24SensorMsg.SetHeaderFrame(frameID);
            }

            if (publishPCL48)
            {
                pcl48Data = new byte[0];
                pcl48Publisher = SimulatorROS2Node.CreatePublisher<sensor_msgs.msg.PointCloud2>(pcl48Topic, qosSettings.GetQoSProfile());
                pcl48RglHandle = lidarSensor.AddFormat(transformUnity2RosHandle, FormatPCL48.GetRGLFields());
                pcl48SensorMsg = FormatPCL48.GetSensorMsg();
                pcl48SensorMsg.SetHeaderFrame(frameID);
            }
        }

        private void OnNewLidarData()
        {
            Profiler.BeginSample("Publish Pointclouds");
            if (publishPCL24)
            {
                int hitCount = lidarSensor.GetDataRaw(pcl24RglHandle, ref pcl24Data, 24);
                PublishFormat(pcl24Publisher, pcl24SensorMsg, pcl24Data, hitCount);
            }

            if (publishPCL48)
            {
                int hitCount = lidarSensor.GetDataRaw(pcl48RglHandle, ref pcl48Data, 48);
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

