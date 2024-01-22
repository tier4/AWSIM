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
        float timeScale = 1.0f;


        #region [Life Cycle]

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

        #endregion

        void Start()
        {
            timeScale = Time.timeScale;
            InvokeRepeating("PublishClock", 1.0f, timeScale/publishHz);
        }

        void Update() 
        {
            if (Mathf.Abs(Time.timeScale - timeScale) > 0.01f)
            {
                timeScale = Time.timeScale;
                OnTimeScaleChanged();
            }
        }

        void OnTimeScaleChanged()
        {
            CancelInvoke();
            InvokeRepeating("PublishClock", 0.0f, timeScale/publishHz);
        }

        void PublishClock()
        {
            SimulatorROS2Node.UpdateROSClockTime(clockMsg.Clock_);
            clockPublisher.Publish(clockMsg);
        }
    }
}
