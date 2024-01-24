using UnityEngine;
using ROS2;
using System.Threading;

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

        Thread clockThread;
        bool isRunning = false;


        #region [Life Cycle]

        void Awake()
        {
            var qos = qosSettings.GetQoSProfile();
            clockPublisher = SimulatorROS2Node.CreatePublisher<rosgraph_msgs.msg.Clock>(topic, qos);
            clockMsg = new rosgraph_msgs.msg.Clock();
        }

        void Start()
        {
            TimeScaleProvider.DoUpdate();
            StartClockThread();
        }

        void OnDestroy()
        {
            StopClockThread();
            SimulatorROS2Node.RemovePublisher<rosgraph_msgs.msg.Clock>(clockPublisher);
        }

        #endregion

        #region [Clock Thread]

        void StartClockThread()
        {
            clockThread = new Thread(UpdateClock)
            {
                Name = "Clock"
            };
            isRunning = true;
            clockThread.Start();
        }

        void StopClockThread()
        {
            isRunning = false;
            if (clockThread != null && clockThread.IsAlive)
            {
                clockThread.Join();
            }
        }

        void UpdateClock()
        {
            while(isRunning)
            {
                Thread.Sleep(1000 / publishHz);
                PublishClock();
            }
        }

        void PublishClock()
        {
            SimulatorROS2Node.UpdateROSClockTime(clockMsg.Clock_);
            clockPublisher.Publish(clockMsg);
        }

        #endregion
    }
}
