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
        [SerializeField, Range(1, 100)] int publishHz;

        IPublisher<rosgraph_msgs.msg.Clock> clockPublisher;
        rosgraph_msgs.msg.Clock clockMsg;
        float timer = 0;

        void PublishClock()
        {
            SimulatorROS2Node.UpdateROSClockTime(clockMsg.Clock_);
            clockPublisher.Publish(clockMsg);
        }

        void Start()
        {
            InvokeRepeating("PublishClock", 1.0f, 1.0f/publishHz);
        }

        // Start is called before the first frame update
        void Awake()
        {
            var qos = qosSettings.GetQoSProfile();
            clockPublisher = SimulatorROS2Node.CreatePublisher<rosgraph_msgs.msg.Clock>(topic, qos);
            clockMsg = new rosgraph_msgs.msg.Clock();
        }


        void OnDestroy()
        {
            SimulatorROS2Node.RemovePublisher<rosgraph_msgs.msg.Clock>(clockPublisher);
        }
    }
}