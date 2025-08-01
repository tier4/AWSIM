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

using UnityEngine;
using ROS2;
using Awsim.Common;

namespace Awsim.Entity
{
    public class GnssRos2Publisher : MonoBehaviour
    {
        [SerializeField] string _poseTopic = "/sensing/gnss/pose";
        [SerializeField] string _poseWithCovarianceStampedTopic = "/sensing/gnss/pose_with_covariance";
        [SerializeField] string _frameID = "/gnss_link";

        [SerializeField]
        QosSettings _qosSettings = new QosSettings(ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
                                                   DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
                                                   HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
                                                   1000);
        [SerializeField] GnssSensor _gnssSensor;

        IPublisher<geometry_msgs.msg.PoseStamped> _posePublisher;
        IPublisher<geometry_msgs.msg.PoseWithCovarianceStamped> _poseWithCovarianceStampedPublisher;
        geometry_msgs.msg.PoseStamped _poseMsg;
        geometry_msgs.msg.PoseWithCovarianceStamped _poseWithCovarianceStampedMsg;

        void Reset()
        {
            var instance = GetComponent<GnssSensor>();
            if (instance != null)
                _gnssSensor = instance;
        }

        public void Initialize()
        {
            // Set callbacks.
            _gnssSensor.OnOutput += Publish;

            // Create msgs.
            _poseMsg = new geometry_msgs.msg.PoseStamped()
            {
                Header = new std_msgs.msg.Header()
                {
                    Frame_id = _frameID,
                },
                Pose = new geometry_msgs.msg.Pose(),
            };

            _poseWithCovarianceStampedMsg = new geometry_msgs.msg.PoseWithCovarianceStamped()
            {
                Header = new std_msgs.msg.Header()
                {
                    Frame_id = _frameID,
                },
                Pose = new geometry_msgs.msg.PoseWithCovariance(),
            };
            for (int i = 0; i < _poseWithCovarianceStampedMsg.Pose.Covariance.Length; i++)
                _poseWithCovarianceStampedMsg.Pose.Covariance[i] = 0;

            // Create publishers.
            var qos = _qosSettings.GetQosProfile();
            _posePublisher = AwsimRos2Node.CreatePublisher<geometry_msgs.msg.PoseStamped>(_poseTopic, qos);
            _poseWithCovarianceStampedPublisher
                = AwsimRos2Node.CreatePublisher<geometry_msgs.msg.PoseWithCovarianceStamped>(_poseWithCovarianceStampedTopic, qos);
        }


        void Publish(GnssSensor.IReadOnlyOutputData outputData)
        {
            // Converts data output from GnssSensor to ROS2 msgs.
            _poseMsg.Pose.Position.X = outputData.Mgrs.Position.x;
            _poseMsg.Pose.Position.Y = outputData.Mgrs.Position.y;
            _poseMsg.Pose.Position.Z = outputData.Mgrs.Position.z;
            _poseWithCovarianceStampedMsg.Pose.Pose.Position.X = outputData.Mgrs.Position.x;
            _poseWithCovarianceStampedMsg.Pose.Pose.Position.Y = outputData.Mgrs.Position.y;
            _poseWithCovarianceStampedMsg.Pose.Pose.Position.Z = outputData.Mgrs.Position.z;

            // Update msg header.
            var poseWithCovarianceStampedHeader = _poseWithCovarianceStampedMsg as MessageWithHeader;
            var poseHeader = _poseMsg as MessageWithHeader;
            AwsimRos2Node.UpdateROSTimestamps(poseHeader, poseWithCovarianceStampedHeader);

            // Publish to ROS2.
            _posePublisher.Publish(_poseMsg);
            _poseWithCovarianceStampedPublisher.Publish(_poseWithCovarianceStampedMsg);
        }

        void OnDestroy()
        {
            AwsimRos2Node.RemovePublisher<geometry_msgs.msg.PoseStamped>(_posePublisher);
            AwsimRos2Node.RemovePublisher<geometry_msgs.msg.PoseWithCovarianceStamped>(_poseWithCovarianceStampedPublisher);
        }
    }
}