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

        private Publisher<sensor_msgs.msg.PointCloud2> _publisherPCL24;
        private Publisher<sensor_msgs.msg.PointCloud2> _publisherPCL48;
        private sensor_msgs.msg.PointCloud2 pcl24;
        private sensor_msgs.msg.PointCloud2 pcl48;
        private LidarSensor lidarSensor;

        private void Start()
        {
            lidarSensor = GetComponent<LidarSensor>();
            lidarSensor.OnOutputData += Publish;

            if (publishPCL24) _publisherPCL24 = SimulatorROS2Node.CreatePublisher<sensor_msgs.msg.PointCloud2>(pcl24Topic, qosSettings.GetQoSProfile());
            if (publishPCL48) _publisherPCL48 = SimulatorROS2Node.CreatePublisher<sensor_msgs.msg.PointCloud2>(pcl48Topic, qosSettings.GetQoSProfile());

            if (publishPCL24) pcl24 = precomputePCL24Message();
            if (publishPCL48) pcl48 = precomputePCL48Message();

            if (publishPCL24) pcl24.SetHeaderFrame(frameID);
            if (publishPCL48) pcl48.SetHeaderFrame(frameID);
        }

        private void Publish(LidarSensor.OutputData data)
        {
            Profiler.BeginSample("Publish Pointclouds");
            if (publishPCL24) PublishFormat(_publisherPCL24, pcl24, data.rosPCL24, data.hitCount);
            if (publishPCL48) PublishFormat(_publisherPCL48, pcl48, data.rosPCL48, data.hitCount);
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
            if(_publisherPCL24 != null) _publisherPCL24.Dispose();
            if(_publisherPCL48 != null) _publisherPCL48.Dispose();
        }

        private sensor_msgs.msg.PointCloud2 precomputeLegacyMessage()
        {
            return new sensor_msgs.msg.PointCloud2
            {
                Header = new std_msgs.msg.Header(), // TO BE FILLED 
                Data = null, // TO BE FILLED
                Is_bigendian = false,
                Width = 0, // TO BE FILLED
                Height = 1,
                Is_dense = true,
                Point_step = 12,
                Row_step = 0, // TO BE FILLED,
                Fields = new[]
                {
                    new sensor_msgs.msg.PointField
                    {
                        Name = "x",
                        Count = 1,
                        Datatype = sensor_msgs.msg.PointField.FLOAT32,
                        Offset = 0,
                    },
                    new sensor_msgs.msg.PointField
                    {
                        Name = "y",
                        Count = 1,
                        Datatype = sensor_msgs.msg.PointField.FLOAT32,
                        Offset = 4,
                    },
                    new sensor_msgs.msg.PointField
                    {
                        Name = "z",
                        Count = 1,
                        Datatype = sensor_msgs.msg.PointField.FLOAT32,
                        Offset = 8,
                    }
                }
            };
        }

        private sensor_msgs.msg.PointCloud2 precomputePCL24Message()
        {
            return new sensor_msgs.msg.PointCloud2()
            {
                Header = new std_msgs.msg.Header(), // TO BE FILLED 
                Data = null, // TO BE FILLED
                Is_bigendian = false,
                Width = 0, // TO BE FILLED
                Height = 1,
                Is_dense = true,
                Point_step = 24,
                Row_step = 0, // TO BE FILLED,
                Fields = new[]
                {
                    new sensor_msgs.msg.PointField
                    {
                        Name = "x",
                        Count = 1,
                        Datatype = sensor_msgs.msg.PointField.FLOAT32,
                        Offset = 0,
                    },
                    new sensor_msgs.msg.PointField
                    {
                        Name = "y",
                        Count = 1,
                        Datatype = sensor_msgs.msg.PointField.FLOAT32,
                        Offset = 4,
                    },
                    new sensor_msgs.msg.PointField
                    {
                        Name = "z",
                        Count = 1,
                        Datatype = sensor_msgs.msg.PointField.FLOAT32,
                        Offset = 8,
                    },
                    new sensor_msgs.msg.PointField
                    {
                        Name = "intensity",
                        Count = 1,
                        Datatype = sensor_msgs.msg.PointField.FLOAT32,
                        Offset = 16,
                    },
                    new sensor_msgs.msg.PointField
                    {
                        Name = "ring",
                        Count = 1,
                        Datatype = sensor_msgs.msg.PointField.UINT16,
                        Offset = 20,
                    }
                }
            };
        }

        private sensor_msgs.msg.PointCloud2 precomputePCL48Message()
        {
            return new sensor_msgs.msg.PointCloud2()
            {
                Header = new std_msgs.msg.Header(), // TO BE FILLED 
                Data = null, // TO BE FILLED
                Is_bigendian = false,
                Width = 0, // TO BE FILLED
                Height = 1,
                Is_dense = true,
                Point_step = 48,
                Row_step = 0, // TO BE FILLED,
                Fields = new[]
                {
                    new sensor_msgs.msg.PointField
                    {
                        Name = "x",
                        Count = 1,
                        Datatype = sensor_msgs.msg.PointField.FLOAT32,
                        Offset = 0,
                    },
                    new sensor_msgs.msg.PointField
                    {
                        Name = "y",
                        Count = 1,
                        Datatype = sensor_msgs.msg.PointField.FLOAT32,
                        Offset = 4,
                    },
                    new sensor_msgs.msg.PointField
                    {
                        Name = "z",
                        Count = 1,
                        Datatype = sensor_msgs.msg.PointField.FLOAT32,
                        Offset = 8,
                    },
                    new sensor_msgs.msg.PointField
                    {
                        Name = "intensity",
                        Count = 1,
                        Datatype = sensor_msgs.msg.PointField.FLOAT32,
                        Offset = 16,
                    },
                    new sensor_msgs.msg.PointField
                    {
                        Name = "ring",
                        Count = 1,
                        Datatype = sensor_msgs.msg.PointField.UINT16,
                        Offset = 20,
                    },
                    new sensor_msgs.msg.PointField
                    {
                        Name = "azimuth",
                        Count = 1,
                        Datatype = sensor_msgs.msg.PointField.FLOAT32,
                        Offset = 24,
                    },
                    new sensor_msgs.msg.PointField
                    {
                        Name = "distance",
                        Count = 1,
                        Datatype = sensor_msgs.msg.PointField.FLOAT32,
                        Offset = 28,
                    },
                    new sensor_msgs.msg.PointField
                    {
                        Name = "return_type",
                        Count = 1,
                        Datatype = sensor_msgs.msg.PointField.UINT8,
                        Offset = 32
                    },
                    new sensor_msgs.msg.PointField
                    {
                        Name = "time_stamp",
                        Count = 1,
                        Datatype = sensor_msgs.msg.PointField.FLOAT64,
                        Offset = 40,
                    }
                }
            };
        }
    }
}

