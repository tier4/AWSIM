// Copyright 2019-2021 Robotec.ai.
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

using UnityEngine;
using System.Threading;

namespace ROS2
{

/// <summary>
/// An example class provided for performance testing of ROS2 communication
/// </summary>
public class ROS2PerformanceTest : MonoBehaviour
{
    public int messageSize = 10000;
    public int rate = 10;
    private int interval_ms = 100;
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<sensor_msgs.msg.PointCloud2> perf_pub;
    sensor_msgs.msg.PointCloud2 msg;
    private bool initialized = false;

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        PrepMessage();
    }

    void OnValidate()
    {
        if (rate < 1)
        {
            interval_ms = 0;
        }
        else
        {
            interval_ms = 1000 / rate;
        }
        PrepMessage();
    }

    private void Publish()
    {
        while(true)
        {
            if (ros2Unity.Ok())
            {
                if (ros2Node == null)
                {
                    ros2Node = ros2Unity.CreateNode("ros2_unity_performance_test_node");
                    perf_pub = ros2Node.CreateSensorPublisher<sensor_msgs.msg.PointCloud2>("perf_chatter");
                }

                var msgWithHeader = msg as MessageWithHeader;
                ros2Node.clock.UpdateROSTimestamp(ref msgWithHeader);
                perf_pub.Publish(msg);
                if (interval_ms > 0)
                {
                    Thread.Sleep(interval_ms);
                }
            }
        }
    }

    void FixedUpdate()
    {
        if (!initialized)
        {
            Thread publishThread = new Thread(() => Publish());
            publishThread.Start();
            initialized = true;
        }
    }

    private void AssignField(ref sensor_msgs.msg.PointField pf, string n, uint off, byte dt, uint count)
    {
        pf.Name = n;
        pf.Offset = off;
        pf.Datatype = dt;
        pf.Count = count;
    }

    private void PrepMessage()
    {
        uint count = (uint)messageSize; //point per message
        uint fieldsSize = 16;
        uint rowSize = count * fieldsSize;
        msg = new sensor_msgs.msg.PointCloud2()
        {
            Height = 1,
            Width = count,
            Is_bigendian = false,
            Is_dense = true,
            Point_step = fieldsSize,
            Row_step = rowSize,
            Data = new byte[rowSize * 1]
        };
        uint pointFieldCount = 4;
        msg.Fields = new sensor_msgs.msg.PointField[pointFieldCount];
        for (int i = 0; i < pointFieldCount; ++i)
        {
            msg.Fields[i] = new sensor_msgs.msg.PointField();
        }

        AssignField(ref msg.Fields[0], "x", 0, 7, 1);
        AssignField(ref msg.Fields[1], "y", 4, 7, 1);
        AssignField(ref msg.Fields[2], "z", 8, 7, 1);
        AssignField(ref msg.Fields[3], "intensity", 12, 7, 1);
        float[] pointsArray = new float[count * msg.Fields.Length];

        var floatIndex = 0;
        for (int i = 0; i < count; ++i)
        {
            float intensity = 100;
            pointsArray[floatIndex++] = 1;
            pointsArray[floatIndex++] = 2;
            pointsArray[floatIndex++] = 3;
            pointsArray[floatIndex++] = intensity;
        }
        System.Buffer.BlockCopy(pointsArray, 0, msg.Data, 0, msg.Data.Length);
        msg.SetHeaderFrame("pc");
    }
}

}  // namespace ROS2
