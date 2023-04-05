
using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{

    /// <summary>
    /// Convert StuckSensor input data from ROS2 and output data to ROS2.
    /// </summary>
    [RequireComponent(typeof(StuckSensor))]
    public class StuckSensorRos2 : MonoBehaviour
    {
        private StuckSensor stuckSensor;
        private IPublisher<std_msgs.msg.Bool> stuckPublisher;
        private ISubscription<autoware_auto_control_msgs.msg.AckermannControlCommand> ackermanControlCommandSubscriber;
        private double subscribedTargetSpeed = 0;

        [Header("ROS Communication Settings")]
        //QoS settings for communication with ROS"
        [SerializeField, Tooltip("QoS settings for communication with ROS")] QoSSettings qosSettings;
        [Header("Input")]
        //The topic to subscribe to the target speed
        [SerializeField, Tooltip("From this topic, speeds are subscribed to calculate the expected distance.")] string ackermannControlCommandTopic = "/control/command/control_cmd";
        [Header("Output")]
        //The topic for publication 'is stuck' state
        [SerializeField, Tooltip("On this topic, the 'is_stuck' state is published (as a std_msgs::Bool)")] string stuckSensorTopic = "/vehicle/status/is_stuck";

        void Start()
        {
            // Get StuckSensor component.
            stuckSensor = GetComponent<StuckSensor>();
            // Set input update callback
            stuckSensor.OnInputData += UpdateSensor;
            // Set output publish callback
            stuckSensor.OnOutputData += Publish;

            stuckPublisher = SimulatorROS2Node.CreatePublisher<std_msgs.msg.Bool>(stuckSensorTopic, qosSettings.GetQoSProfile());
            ackermanControlCommandSubscriber
            = SimulatorROS2Node.CreateSubscription<autoware_auto_control_msgs.msg.AckermannControlCommand>(
                ackermannControlCommandTopic, msg => { subscribedTargetSpeed = msg.Longitudinal.Speed; }, qosSettings.GetQoSProfile());

        }

        void UpdateSensor(StuckSensor.InputData inputData)
        {
            inputData.TargetSpeed = subscribedTargetSpeed;
        }

        void Publish(StuckSensor.OutputData outputData)
        {
            stuckPublisher.Publish(new std_msgs.msg.Bool() { Data = outputData.IsStuck, });
        }

        void OnDestroy()
        {
            SimulatorROS2Node.RemovePublisher<std_msgs.msg.Bool>(stuckPublisher);
            SimulatorROS2Node.RemoveSubscription<autoware_auto_control_msgs.msg.AckermannControlCommand>(ackermanControlCommandSubscriber);
        }
    }

}

