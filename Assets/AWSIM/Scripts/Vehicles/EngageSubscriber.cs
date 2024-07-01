using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// Subscribe to the "/vehicle/engage" topic and change the vehicle's control mode to AUTONOMOUS.
    /// </summary>
    public class EngageSubscriber : MonoBehaviour
    {
        [SerializeField] string engageTopic = "/vehicle/engage";
        [SerializeField] QoSSettings qosSettings = new QoSSettings();
        ISubscription<autoware_vehicle_msgs.msg.Engage> engageSubscriber;

        [SerializeField] VehicleOverrideInputManager vehicleOverrideInputManager;
        void Reset()
        {
            qosSettings.ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE;
            qosSettings.DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
            qosSettings.HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST;
            qosSettings.Depth = 1;
        }

        void OnEnable()
        {
            var qos = qosSettings.GetQoSProfile();

            engageSubscriber = SimulatorROS2Node.CreateSubscription<autoware_vehicle_msgs.msg.Engage>(
                engageTopic, msg =>
                {
                    if (msg.Engage_)
                    {
                        vehicleOverrideInputManager.ChangeControlModeToAUTONOMOUS();
                    }
                }, qos);
        }
    }
}