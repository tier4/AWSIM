using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// When the OnCollisionEnter() callback fires, the topic is published in Ros2.
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    public class OnCollisionRos2Publisher : MonoBehaviour
    {
        /// <summary>
        /// Topic name.
        /// </summary>
        public string topic = "awsim/ground_truth/on_collision";

        /// <summary>
        /// Frame ID.
        /// </summary>
        public string frameId = "base_link";

        /// <summary>
        /// QoS settings.
        /// </summary>
        public QoSSettings qosSettings = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 1,
        };

        IPublisher<std_msgs.msg.Bool> publisher;
        std_msgs.msg.Bool msg;

        void Start()
        {
            // Create msg.
            msg = new std_msgs.msg.Bool()
            {
                Data = true,
            };

            // Get qos.
            var qos = qosSettings.GetQoSProfile();

            // Create publisher.
            publisher = SimulatorROS2Node.CreatePublisher<std_msgs.msg.Bool>(topic, qos);
        }

        void OnCollisionEnter(Collision collision)
        {
            publisher.Publish(msg);
        }
    }
}