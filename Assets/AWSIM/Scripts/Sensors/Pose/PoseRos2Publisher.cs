using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// Convert the data output from PoseSensor to ROS2 msg and Publish.
    /// </summary>
    [RequireComponent(typeof(PoseRos2Publisher))]
    public class PoseRos2Publisher : MonoBehaviour
    {
        /// <summary>
        /// Topic name in pose msg.
        /// </summary>
        public string Topic = "/awsim/ground_truth/vehicle/pose";

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

        IPublisher<geometry_msgs.msg.PoseStamped> poseStampedPublisher;
        geometry_msgs.msg.PoseStamped msg;
        PoseSensor sensor;

        // Start is called before the first frame update
        void Start()
        {
            // Get GnssSensor component.
            sensor = GetComponent<PoseSensor>();

            // Set callback.
            sensor.OnOutputData += Publish;

            // Create msg.
            msg = new geometry_msgs.msg.PoseStamped()
            {
                Header = new std_msgs.msg.Header()
                {
                    Frame_id = FrameID,
                },
                Pose = new geometry_msgs.msg.Pose(),
            };

            // Create publisher.
            var qos = QosSettings.GetQoSProfile();
            poseStampedPublisher = SimulatorROS2Node.CreatePublisher<geometry_msgs.msg.PoseStamped>(Topic, qos);
        }

        void Publish(PoseSensor.OutputData outputData)
        {
            // Converts data output from GnssSensor to ROS2 msg
            var rosPosition = outputData.Position;
            var rosRotation = outputData.Rotation;

            msg.Pose.Position.X = rosPosition.x;
            msg.Pose.Position.Y = rosPosition.y;
            msg.Pose.Position.Z = rosPosition.z;

            msg.Pose.Orientation.X = rosRotation.x;
            msg.Pose.Orientation.Y = rosRotation.y;
            msg.Pose.Orientation.Z = rosRotation.z;
            msg.Pose.Orientation.W = rosRotation.w;

            // Update msg header.
            var header = msg as MessageWithHeader;
            SimulatorROS2Node.UpdateROSTimestamp(ref header);

            // Publish to ROS2.
            poseStampedPublisher.Publish(msg);
        }

        void OnDestroy()
        {
            SimulatorROS2Node.RemovePublisher<geometry_msgs.msg.PoseStamped>(poseStampedPublisher);
        }
    }
}