using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// Publish clock to ros2.
    /// </summary>
    public class ClockPublisher : MonoBehaviour
    {
        [SerializeField] string topic;
        [SerializeField] QoSSettings qosSettings;
        [SerializeField, Range(0, 100)] int publishHz;

        IPublisher<rosgraph_msgs.msg.Clock> clockPublisher;
        rosgraph_msgs.msg.Clock clockMsg;
        float timer = 0;

        void Reset()
        {
            topic = "/clock";
            qosSettings = new QoSSettings()
            {
                ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT,
                DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
                HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
                Depth = 1,
            };
            publishHz = 100;
        }

        // Start is called before the first frame update
        void Awake()
        {
            var qos = qosSettings.GetQoSProfile();
            clockPublisher = SimulatorROS2Node.CreatePublisher<rosgraph_msgs.msg.Clock>(topic, qos);
            clockMsg = new rosgraph_msgs.msg.Clock();
        }

        void FixedUpdate()
        {
            timer += Time.deltaTime;
            var interval = 1.0f / publishHz;
            interval -= 0.00001f;       // Allow for accuracy errors.
            if (timer < interval)
                return;
            timer = 0;

            SimulatorROS2Node.UpdateROSClockTime(clockMsg.Clock_);
            clockPublisher.Publish(clockMsg);
        }

        void OnDestroy()
        {
            SimulatorROS2Node.RemovePublisher<rosgraph_msgs.msg.Clock>(clockPublisher);
        }
    }
}