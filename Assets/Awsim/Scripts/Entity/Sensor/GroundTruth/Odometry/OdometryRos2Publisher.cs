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

using Awsim.Common;
using Awsim.Entity;
using ROS2;
using UnityEngine;

public class OdometryRos2Publisher : MonoBehaviour
{
    [SerializeField] string _topic = "/awsim/ground_truth/localization/kinematic_state";
    [SerializeField] string _frameId = "base_link";
    [SerializeField] OdometrySensor _odometrySensor;

    [SerializeField]
    QosSettings _qosSettings = new QosSettings(ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                               DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
                                               HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
                                               1);

    IPublisher<nav_msgs.msg.Odometry> _odometryPublisher;
    nav_msgs.msg.Odometry _msg;
    geometry_msgs.msg.PoseWithCovariance _poseMsg;
    geometry_msgs.msg.TwistWithCovariance _twistMsg;
    std_msgs.msg.String _frameIdMsg;

    public void Initialize()
    {
        _odometrySensor.OnOutput += Publish;

        _msg = new nav_msgs.msg.Odometry()
        {
            Header = new std_msgs.msg.Header()
            {
                Frame_id = _frameId,
            },
            Child_frame_id = "",
            Pose = new geometry_msgs.msg.PoseWithCovariance(),
            Twist = new geometry_msgs.msg.TwistWithCovariance(),
        };

        var qos = _qosSettings.GetQosProfile();
        _odometryPublisher = AwsimRos2Node.CreatePublisher<nav_msgs.msg.Odometry>(_topic, qos);
    }

    void Publish(OdometrySensor.IReadOnlyOutputData outputData)
    {
        // Converts data output from Pose to ROS2 msg
        var rosPosition = outputData.Position;
        var rosRotation = outputData.Rotation;

        // TODO: Add double[36] covariance
        _msg.Pose.Pose.Position.X = rosPosition.x;
        _msg.Pose.Pose.Position.Y = rosPosition.y;
        _msg.Pose.Pose.Position.Z = rosPosition.z;

        _msg.Pose.Pose.Orientation.X = rosRotation.x;
        _msg.Pose.Pose.Orientation.Y = rosRotation.y;
        _msg.Pose.Pose.Orientation.Z = rosRotation.z;
        _msg.Pose.Pose.Orientation.W = rosRotation.w;

        // Converts data output from Twist to ROS2 msg
        var rosLinearVelocity = Ros2Utility.UnityToRos2Position(outputData.LinearVelocity);
        var rosAngularVelocity = Ros2Utility.UnityToRos2AngularVelocity(outputData.AngularVelocity);
        _msg.Twist.Twist.Linear.X = rosLinearVelocity.x;
        _msg.Twist.Twist.Linear.Y = rosLinearVelocity.y;
        _msg.Twist.Twist.Linear.Z = rosLinearVelocity.z;

        _msg.Twist.Twist.Angular.X = rosAngularVelocity.x;
        _msg.Twist.Twist.Angular.Y = rosAngularVelocity.y;
        _msg.Twist.Twist.Angular.Z = rosAngularVelocity.z;

        // Add covariance 6x6
        const int size = 6;
        for (int i = 0; i < size; i++)
        {
            _msg.Pose.Covariance[i * size + i] = 1;
            _msg.Twist.Covariance[i * size + i] = 1;
        }

        // Update msg header.
        var header = _msg as MessageWithHeader;
        AwsimRos2Node.UpdateROSTimestamp(ref header);

        _msg.Child_frame_id = "base_link";

        // Publish to ROS2.
        _odometryPublisher.Publish(_msg);
    }

    void OnDestroy()
    {
        AwsimRos2Node.RemovePublisher<nav_msgs.msg.Odometry>(_odometryPublisher);
    }
}
