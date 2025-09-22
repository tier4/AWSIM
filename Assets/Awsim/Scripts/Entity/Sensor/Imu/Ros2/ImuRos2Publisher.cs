// Copyright 2025 TIER IV, Inc.
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

using ROS2;
using UnityEngine;
using Awsim.Common;

namespace Awsim.Entity
{
    public class ImuRos2Publisher : MonoBehaviour
    {
        public string Topic { get => _topic; }
        public string FrameId { get => _frameId; }
        public QosSettings QosSettings { get => _qosSettings; }

        [SerializeField] string _topic = "/sensing/imu/tamagawa/imu_raw";
        [SerializeField] string _frameId = "tamagawa/imu_link";
        [SerializeField]
        QosSettings _qosSettings = new QosSettings(ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
                                                                    DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
                                                                    HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
                                                                    1000);

        [SerializeField] ImuSensor _imuSensor = null;
        IPublisher<sensor_msgs.msg.Imu> _imuPublisher = null;
        sensor_msgs.msg.Imu _imuMsg = null;

        void Reset()
        {
            var instance = GetComponent<ImuSensor>();
            if (instance != null)
                _imuSensor = instance;
        }

        public void Initialize()
        {
            _imuSensor.OnOutput += Publish;

            _imuMsg = new sensor_msgs.msg.Imu()
            {
                Linear_acceleration = new geometry_msgs.msg.Vector3(),
                Angular_velocity = new geometry_msgs.msg.Vector3(),
                Orientation = new geometry_msgs.msg.Quaternion()
                {
                    W = 1,
                    X = 0,
                    Y = 0,
                    Z = 0,
                },
                Header = new std_msgs.msg.Header()
                {
                    Frame_id = _frameId,
                }
            };

            // Set covariances to 0.
            for (int i = 0; i < _imuMsg.Angular_velocity_covariance.Length; i++)
                _imuMsg.Angular_velocity_covariance[i] = 0;
            for (int i = 0; i < _imuMsg.Linear_acceleration_covariance.Length; i++)
                _imuMsg.Linear_acceleration_covariance[i] = 0;
            for (int i = 0; i < _imuMsg.Orientation_covariance.Length; i++)
                _imuMsg.Orientation_covariance[i] = 0;

            // Create publisher.
            _imuPublisher = AwsimRos2Node.CreatePublisher<sensor_msgs.msg.Imu>(_topic, _qosSettings.GetQosProfile());
        }

        public void Initialize(string topic, string frameId, QosSettings qosSettings)
        {
            _topic = topic;
            _frameId = frameId;
            _qosSettings = qosSettings;

            Initialize();
        }

        void Publish(ImuSensor.IReadOnlyOutputData outputData)
        {
            var rosLinearAcceleration = Ros2Utility.UnityToRos2Position(outputData.LinearAcceleration);
            var rosAngularVelocity = Ros2Utility.UnityToRos2AngularVelocity(outputData.AngularVelocity);

            _imuMsg.Linear_acceleration.X = rosLinearAcceleration.x;
            _imuMsg.Linear_acceleration.Y = rosLinearAcceleration.y;
            _imuMsg.Linear_acceleration.Z = rosLinearAcceleration.z;
            _imuMsg.Angular_velocity.X = rosAngularVelocity.x;
            _imuMsg.Angular_velocity.Y = rosAngularVelocity.y;
            _imuMsg.Angular_velocity.Z = rosAngularVelocity.z;

            var header = _imuMsg as MessageWithHeader;
            AwsimRos2Node.UpdateROSTimestamp(ref header);

            _imuPublisher.Publish(_imuMsg);
        }

        void OnDestroy()
        {
            AwsimRos2Node.RemovePublisher<sensor_msgs.msg.Imu>(_imuPublisher);
        }
    }
}