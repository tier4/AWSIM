using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// Convert the data output from Odometry Sensor to ROS2 msg and Publish.
    /// </summary>
    [RequireComponent(typeof(OdometryRos2Publisher))]
    public class OdometryRos2Publisher : MonoBehaviour
    {
        /// <summary>
        /// Topic name in pose msg.
        /// </summary>
        public string Topic = "/awsim/ground_truth/localization/kinematic_state";

        /// <summary>
        /// Pose sensor frame id.
        /// </summary>
        public string FrameID = "base_link";

        /// <summary>
        /// QoS settings.
        /// </summary>
        public QoSSettings QosSettings = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 1,
        };

        IPublisher<nav_msgs.msg.Odometry> odometryPublisher;
        nav_msgs.msg.Odometry msg;
        geometry_msgs.msg.PoseWithCovariance pose;
        geometry_msgs.msg.TwistWithCovariance twist;
        std_msgs.msg.String frame_id;
        OdometrySensor sensor;

        // Start is called before the first frame update
        void Start()
        {
            // Get OdometrySensor component.
            sensor = GetComponent<OdometrySensor>();

            // Set callback.
            sensor.OnOutputData += Publish;

            // Create msg.
            msg = new nav_msgs.msg.Odometry()
            {
                Header = new std_msgs.msg.Header()
                {
                    Frame_id = FrameID,
                },
                Child_frame_id = "",
                Pose = new geometry_msgs.msg.PoseWithCovariance(),
                Twist = new geometry_msgs.msg.TwistWithCovariance(),
            };

            // Create publisher.
            var qos = QosSettings.GetQoSProfile();
            odometryPublisher = SimulatorROS2Node.CreatePublisher<nav_msgs.msg.Odometry>(Topic, qos);
        }

        void Publish(OdometrySensor.OutputData outputData)
        {
            // Converts data output from Pose to ROS2 msg
            var rosPosition = outputData.Position;
            var rosRotation = outputData.Rotation;

            // TODO: Add double[36] covariance
            msg.Pose.Pose.Position.X = rosPosition.x;
            msg.Pose.Pose.Position.Y = rosPosition.y;
            msg.Pose.Pose.Position.Z = rosPosition.z;

            msg.Pose.Pose.Orientation.X = rosRotation.x;
            msg.Pose.Pose.Orientation.Y = rosRotation.y;
            msg.Pose.Pose.Orientation.Z = rosRotation.z;
            msg.Pose.Pose.Orientation.W = rosRotation.w;

             // Converts data output from Twist to ROS2 msg
            var rosLinearVelocity = ROS2Utility.UnityToRosPosition(outputData.linearVelocity);
            var rosAngularVelocity = ROS2Utility.UnityToRosAngularVelocity(outputData.angularVelocity);
            msg.Twist.Twist.Linear.X = rosLinearVelocity.x;
            msg.Twist.Twist.Linear.Y = rosLinearVelocity.y;
            msg.Twist.Twist.Linear.Z = rosLinearVelocity.z;

            msg.Twist.Twist.Angular.X = rosAngularVelocity.x;
            msg.Twist.Twist.Angular.Y = rosAngularVelocity.y;
            msg.Twist.Twist.Angular.Z = rosAngularVelocity.z;

            // Add covariance 6x6
            const int size = 6;
            for (int i = 0; i < size; i++)
            {
                msg.Pose.Covariance[i * size + i] = 1;
                msg.Twist.Covariance[i * size + i] = 1;
            }

            // Update msg header.
            var header = msg as MessageWithHeader;
            SimulatorROS2Node.UpdateROSTimestamp(ref header);

            msg.Child_frame_id = "base_link";

            // Publish to ROS2.
            odometryPublisher.Publish(msg);
        }

        void OnDestroy()
        {
            SimulatorROS2Node.RemovePublisher<nav_msgs.msg.Odometry>(odometryPublisher);
        }
    }
}