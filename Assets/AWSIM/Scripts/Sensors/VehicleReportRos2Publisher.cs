using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// Output Autoware's Vehicle status topic.
    /// </summary>
    public class VehicleReportRos2Publisher : MonoBehaviour
    {
        /// <summary>
        /// Topic name of ControlModeReport msg.
        /// </summary>
        [SerializeField] string controlModeReportTopic = "/vehicle/status/control_mode";

        /// <summary>
        /// Topic name of GearReport msg.
        /// </summary>
        [SerializeField] string gearReportTopic = "/vehicle/status/gear_status";

        /// <summary>
        /// Topic name of SteeringReport msg.
        /// </summary>
        [SerializeField] string steeringReportTopic = "/vehicle/status/steering_status";

        /// <summary>
        /// Topic name of TurnIndicatorsReport msg.
        /// </summary>
        [SerializeField] string turnIndicatorsReportTopic = "/vehicle/status/turn_indicators_status";

        /// <summary>
        /// Topic name of hazardLightsReport msg.
        /// </summary>
        [SerializeField] string hazardLightsReportTopic = "/vehicle/status/hazard_lights_status";

        /// <summary>
        /// Topic name of VelocityReport msg.
        /// </summary>
        [SerializeField] string velocityReportTopic = "/vehicle/status/velocity_status";

        /// <summary>
        /// VelocityRepor frame id.
        /// </summary>
        [SerializeField] string frameId = "base_link";

        /// <summary>
        /// Hz to publish to ROS2.
        /// </summary>
        [Range(1, 60)]
        [SerializeField] int PublishHz = 30;

        [SerializeField] QoSSettings qosSettings;

        [SerializeField] Vehicle vehicle;

        // msgs.
        autoware_auto_vehicle_msgs.msg.ControlModeReport controlModeReportMsg;
        autoware_auto_vehicle_msgs.msg.GearReport gearReportMsg;
        autoware_auto_vehicle_msgs.msg.SteeringReport steeringReportMsg;
        autoware_auto_vehicle_msgs.msg.TurnIndicatorsReport turnIndicatorsReportMsg;
        autoware_auto_vehicle_msgs.msg.HazardLightsReport hazardLightsReportMsg;
        autoware_auto_vehicle_msgs.msg.VelocityReport velocityReportMsg;

        // publisher.
        IPublisher<autoware_auto_vehicle_msgs.msg.ControlModeReport> controlModeReportPublisher;
        IPublisher<autoware_auto_vehicle_msgs.msg.GearReport> gearReportPublisher;
        IPublisher<autoware_auto_vehicle_msgs.msg.SteeringReport> steeringReportPublisher;
        IPublisher<autoware_auto_vehicle_msgs.msg.TurnIndicatorsReport> turnIndicatorsReportPublisher;
        IPublisher<autoware_auto_vehicle_msgs.msg.HazardLightsReport> hazardLightsReportPublisher;
        IPublisher<autoware_auto_vehicle_msgs.msg.VelocityReport> velocityReportPublisher;

        bool initialized = false;
        float timer;

        void Start()
        {
            // Create publisher.
            var qos = qosSettings.GetQoSProfile();
            controlModeReportPublisher = SimulatorROS2Node.CreatePublisher<autoware_auto_vehicle_msgs.msg.ControlModeReport>(controlModeReportTopic, qos);
            gearReportPublisher = SimulatorROS2Node.CreatePublisher<autoware_auto_vehicle_msgs.msg.GearReport>(gearReportTopic, qos);
            steeringReportPublisher = SimulatorROS2Node.CreatePublisher<autoware_auto_vehicle_msgs.msg.SteeringReport>(steeringReportTopic, qos);
            turnIndicatorsReportPublisher = SimulatorROS2Node.CreatePublisher<autoware_auto_vehicle_msgs.msg.TurnIndicatorsReport>(turnIndicatorsReportTopic, qos);
            hazardLightsReportPublisher = SimulatorROS2Node.CreatePublisher<autoware_auto_vehicle_msgs.msg.HazardLightsReport>(hazardLightsReportTopic, qos);
            velocityReportPublisher = SimulatorROS2Node.CreatePublisher<autoware_auto_vehicle_msgs.msg.VelocityReport>(velocityReportTopic, qos);

            // Create msg.
            controlModeReportMsg = new autoware_auto_vehicle_msgs.msg.ControlModeReport();
            gearReportMsg = new autoware_auto_vehicle_msgs.msg.GearReport();
            steeringReportMsg = new autoware_auto_vehicle_msgs.msg.SteeringReport();
            turnIndicatorsReportMsg = new autoware_auto_vehicle_msgs.msg.TurnIndicatorsReport();
            hazardLightsReportMsg = new autoware_auto_vehicle_msgs.msg.HazardLightsReport();
            velocityReportMsg = new autoware_auto_vehicle_msgs.msg.VelocityReport()
            {
                Header = new std_msgs.msg.Header()
                {
                    Frame_id = frameId,
                }
            };

            initialized = true;
        }

        void FixedUpdate()
        {
            if (initialized == false)
                return;

            // Update timer.
            timer += Time.deltaTime;

            // Matching publish to hz.
            var interval = 1.0f / PublishHz;
            interval -= 0.00001f;       // Allow for accuracy errors.
            if (timer < interval)
                return;
            timer = 0;

            // ControlModeReport
            controlModeReportMsg.Mode = autoware_auto_vehicle_msgs.msg.ControlModeReport.AUTONOMOUS;

            // GearReport
            gearReportMsg.Report = VehicleROS2Utility.UnityToRosShift(vehicle.AutomaticShiftInput);

            // SteeringReport
            steeringReportMsg.Steering_tire_angle = -1 * vehicle.SteerAngle * Mathf.Deg2Rad;

            // TurnIndicatorsReport
            turnIndicatorsReportMsg.Report = VehicleROS2Utility.UnityToRosTurnSignal(vehicle.SignalInput);

            // HazardLightsReport
            hazardLightsReportMsg.Report = VehicleROS2Utility.UnityToRosHazard(vehicle.SignalInput);

            // VelocityReport
            var rosLinearVelocity = ROS2Utility.UnityToRosPosition(vehicle.LocalVelocity);
            var rosAngularVelocity = ROS2Utility.UnityToRosPosition(-vehicle.AngularVelocity);
            velocityReportMsg.Longitudinal_velocity = rosLinearVelocity.x;
            velocityReportMsg.Lateral_velocity = rosLinearVelocity.y;
            velocityReportMsg.Heading_rate = rosAngularVelocity.z;

            // Update Stamp
            var time = SimulatorROS2Node.GetCurrentRosTime();
            controlModeReportMsg.Stamp = time;
            gearReportMsg.Stamp = time;
            steeringReportMsg.Stamp = time;
            turnIndicatorsReportMsg.Stamp = time;
            hazardLightsReportMsg.Stamp = time;
            var velocityReportMsgHeader = velocityReportMsg as MessageWithHeader;
            SimulatorROS2Node.UpdateROSTimestamp(ref velocityReportMsgHeader);

            // publish
            controlModeReportPublisher.Publish(controlModeReportMsg);
            gearReportPublisher.Publish(gearReportMsg);
            steeringReportPublisher.Publish(steeringReportMsg);
            turnIndicatorsReportPublisher.Publish(turnIndicatorsReportMsg);
            hazardLightsReportPublisher.Publish(hazardLightsReportMsg);
            velocityReportPublisher.Publish(velocityReportMsg);
        }

        void OnDestroy()
        {
            SimulatorROS2Node.RemovePublisher<autoware_auto_vehicle_msgs.msg.ControlModeReport>(controlModeReportPublisher);
            SimulatorROS2Node.RemovePublisher<autoware_auto_vehicle_msgs.msg.GearReport>(gearReportPublisher);
            SimulatorROS2Node.RemovePublisher<autoware_auto_vehicle_msgs.msg.SteeringReport>(steeringReportPublisher);
            SimulatorROS2Node.RemovePublisher<autoware_auto_vehicle_msgs.msg.TurnIndicatorsReport>(turnIndicatorsReportPublisher);
            SimulatorROS2Node.RemovePublisher<autoware_auto_vehicle_msgs.msg.HazardLightsReport>(hazardLightsReportPublisher);
            SimulatorROS2Node.RemovePublisher<autoware_auto_vehicle_msgs.msg.VelocityReport>(velocityReportPublisher);
        }
    }
}