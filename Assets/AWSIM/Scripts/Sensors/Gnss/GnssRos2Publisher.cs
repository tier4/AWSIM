using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// Convert the data output from GnssSensor to ROS2 msg and Publish.
    /// </summary>
    [RequireComponent(typeof(GnssSensor))]
    public class GnssRos2Publisher : MonoBehaviour
    {
        /// <summary>
        /// Topic name in pose msg.
        /// </summary>
        public string poseTopic = "/sensing/gnss/pose";

        /// <summary>
        /// Topic name in poseWithCovarianceStamped msg.
        /// </summary>
        public string poseWithCovarianceStampedTopic = "/sensing/gnss/pose_with_covariance";

        /// <summary>
        /// Gnss sensor frame id.
        /// </summary>
        public string frameId = "gnss_link";

        /// <summary>
        /// QoS settings.
        /// </summary>
        public QoSSettings qosSettings;

        IPublisher<geometry_msgs.msg.PoseStamped> posePublisher;
        IPublisher<geometry_msgs.msg.PoseWithCovarianceStamped> poseWithCovarianceStampedPublisher;
        geometry_msgs.msg.PoseStamped poseMsg;
        geometry_msgs.msg.PoseWithCovarianceStamped poseWithCovarianceStampedMsg;
        GnssSensor gnssSensor;

        void Start()
        {
            // Get GnssSensor component.
            gnssSensor = GetComponent<GnssSensor>();

            // Set callback.
            gnssSensor.OnOutputData += Publish;

            // Create msg.
            poseMsg = new geometry_msgs.msg.PoseStamped()
            {
                Header = new std_msgs.msg.Header()
                {
                    Frame_id = frameId,
                },
                Pose = new geometry_msgs.msg.Pose(),
            };
            poseWithCovarianceStampedMsg = new geometry_msgs.msg.PoseWithCovarianceStamped()
            {
                Header = new std_msgs.msg.Header()
                {
                    Frame_id = frameId,
                },
                Pose = new geometry_msgs.msg.PoseWithCovariance(),
            };
            for (int i = 0; i < poseWithCovarianceStampedMsg.Pose.Covariance.Length; i++)
                poseWithCovarianceStampedMsg.Pose.Covariance[i] = 0;

            // Create publisher.
            var qos = qosSettings.GetQoSProfile();
            posePublisher = SimulatorROS2Node.CreatePublisher<geometry_msgs.msg.PoseStamped>(poseTopic, qos);
            poseWithCovarianceStampedPublisher = SimulatorROS2Node.CreatePublisher<geometry_msgs.msg.PoseWithCovarianceStamped>(poseWithCovarianceStampedTopic, qos);
        }

        void Publish(GnssSensor.OutputData outputData)
        {
            // Converts data output from GnssSensor to ROS2 msg
            poseMsg.Pose.Position.X = outputData.MgrsPosition.x;
            poseMsg.Pose.Position.Y = outputData.MgrsPosition.y;
            poseMsg.Pose.Position.Z = outputData.MgrsPosition.z;
            poseWithCovarianceStampedMsg.Pose.Pose.Position.X = outputData.MgrsPosition.x;
            poseWithCovarianceStampedMsg.Pose.Pose.Position.Y = outputData.MgrsPosition.y;
            poseWithCovarianceStampedMsg.Pose.Pose.Position.Z = outputData.MgrsPosition.z;

            // Update msg header.
            var poseWithCovarianceStampedHeader = poseWithCovarianceStampedMsg as MessageWithHeader;
            SimulatorROS2Node.UpdateROSTimestamp(ref poseWithCovarianceStampedHeader);
            var poseHeader = poseMsg as MessageWithHeader;
            SimulatorROS2Node.UpdateROSTimestamp(ref poseHeader);

            // Publish to ROS2.
            posePublisher.Publish(poseMsg);
            poseWithCovarianceStampedPublisher.Publish(poseWithCovarianceStampedMsg);
        }

        void OnDestroy()
        {
            SimulatorROS2Node.RemovePublisher<geometry_msgs.msg.PoseStamped>(posePublisher);
            SimulatorROS2Node.RemovePublisher<geometry_msgs.msg.PoseWithCovarianceStamped>(poseWithCovarianceStampedPublisher);
        }
    }
}