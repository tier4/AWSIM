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

using System.Threading;
using UnityEngine;
using ROS2;

namespace Awsim.Common
{
    /// <summary>
    /// Clock publisher class for Ros2.
    /// Clock publishing runs in a separate thread from unity. 
    /// Therefore, it is possible to publish at a higher hz than the main thread of unity.
    /// </summary>
    public class ClockRos2Publisher : MonoBehaviour
    {
        /// <summary>
        /// Clock ros2 topic name.
        /// </summary>
        public string Topic { get => _topic; }

        /// <summary>
        /// Clcok ros2 topic publish hz.
        /// Can be set higher value than the Unity update cycle because it is issued in a separate thread from the unity thread.
        /// </summary>
        public int PublishHz { get => _publishHz; }

        /// <summary>
        /// Ros2 quality of service settings.
        /// </summary>
        public QosSettings QosSettings { get => _qosSettings; }

        [SerializeField] string _topic = "/clock";
        [SerializeField] int _publishHz = 100;
        [SerializeField]
        QosSettings _qosSettings = new QosSettings(ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
                                                   DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
                                                   HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
                                                   1);

        IPublisher<rosgraph_msgs.msg.Clock> _clockPublisher = null;
        rosgraph_msgs.msg.Clock _clockMsg = null;
        Thread _clockThread = null;
        bool _isRunning = false;

        /// <summary>
        /// Initialize clock publisher. Start clock publish thread.
        /// </summary>
        public void Initialize()
        {
            // Initialize publisher and msg.
            var qos = _qosSettings.GetQosProfile();
            _clockPublisher = AwsimRos2Node.CreatePublisher<rosgraph_msgs.msg.Clock>(_topic, qos);
            _clockMsg = new rosgraph_msgs.msg.Clock();

            // Sync unity time.
            ThreadSafeTime.SyncTimeAsDouble();
            ThreadSafeTime.SyncTimeScale();

            // Start thread.
            StartClockThread();
        }

        /// <summary>
        /// Initialize clock publisher. Start clock publish thread.
        /// </summary>
        /// <param name="topic">Ros2 topic name.</param>
        /// <param name="qosSettings">Ros2 QoS settings.</param>
        /// <param name="publishHz">Ros2 topic publishing hz.</param>
        public void Initialize(string topic, QosSettings qosSettings, int publishHz)
        {
            _topic = topic;
            _qosSettings = qosSettings;
            _publishHz = publishHz;

            Initialize();
        }

        /// <summary>
        /// Synchronize Time.timeAsDouble from the Unity main thread.
        /// </summary>
        public void OnUpdate()
        {
            ThreadSafeTime.SyncTimeAsDouble();
        }

        void OnDestroy()
        {
            StopClockThread();
            AwsimRos2Node.RemovePublisher<rosgraph_msgs.msg.Clock>(_clockPublisher);
        }

        void StartClockThread()
        {
            _clockThread = new Thread(UpdateClock)
            {
                Name = "Clock"
            };
            _isRunning = true;
            _clockThread.Start();
        }

        void StopClockThread()
        {
            _isRunning = false;
            if (_clockThread != null && _clockThread.IsAlive)
            {
                _clockThread.Join();
            }
        }

        void UpdateClock()
        {
            while (_isRunning)
            {
                Thread.Sleep(1000 / _publishHz);
                PublishClock();
            }
        }

        void PublishClock()
        {
            AwsimRos2Node.UpdateROSClockTime(_clockMsg.Clock_);
            _clockPublisher.Publish(_clockMsg);
        }
    }
}