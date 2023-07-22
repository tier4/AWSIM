using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// This class subscribes to the vehicleCommand and turnSignal  msg output from Autoware to ROS, 
    /// and after converting the msg, it inputs it to the Vehicle class of E2ESimualtor.
    /// </summary>
    [RequireComponent(typeof(EZ10Vehicle))]
    public class EZ10VehicleRosInput : MonoBehaviour
    {
        [SerializeField] string turnIndicatorsCommandTopic = "/control/command/turn_indicators_cmd";
        [SerializeField] string hazardLightsCommandTopic = "/control/command/hazard_lights_cmd";
        [SerializeField] string ackermannControlCommandTopic = "/control/command/control_cmd";
        [SerializeField] string gearCommandTopic = "/control/command/gear_cmd";
        [SerializeField] string vehicleEmergencyStampedTopic = "/control/command/emergency_cmd";

        [SerializeField] QoSSettings qosSettings = new QoSSettings();
        [SerializeField] EZ10Vehicle vehicle;

        // subscribers.
        ISubscription<autoware_auto_vehicle_msgs.msg.TurnIndicatorsCommand> turnIndicatorsCommandSubscriber;
        ISubscription<autoware_auto_vehicle_msgs.msg.HazardLightsCommand> hazardLightsCommandSubscriber;
        ISubscription<autoware_auto_control_msgs.msg.AckermannControlCommand> ackermanControlCommandSubscriber;
        ISubscription<autoware_auto_vehicle_msgs.msg.GearCommand> gearCommandSubscriber;
        ISubscription<tier4_vehicle_msgs.msg.VehicleEmergencyStamped> vehicleEmergencyStampedSubscriber;

        // Latest Emergency value.
        // If emergency is true, emergencyDeceleration is applied to the vehicle's deceleration.
        // TODO: In case of reverse gear?
        bool isEmergency = false;
        float emergencyDeceleration = -3.0f; // m/s^2

        // Latest value of TurnSignals.
        // HAZARD and LEFT/RIGHT are different msgs in Autoware.universe.
        // Priority : HAZARD > LEFT/RIGHT > NONE
        EZ10Vehicle.TurnSignal turnIndicatorsSignal = EZ10Vehicle.TurnSignal.NONE;
        EZ10Vehicle.TurnSignal hazardLightsSignal = EZ10Vehicle.TurnSignal.NONE;
        EZ10Vehicle.TurnSignal input = EZ10Vehicle.TurnSignal.NONE;


        void Reset()
        {
            if (vehicle == null)
                vehicle = GetComponent<EZ10Vehicle>();

            // initialize default QoS params.
            qosSettings.ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE;
            qosSettings.DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
            qosSettings.HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST;
            qosSettings.Depth = 1;
        }

        /// <summary>
        /// Processes the TurnSignal to be applied to the vehicle from the latest turnIndicatorsSignal and hazardLightsSignal values.
        /// Priority : HAZARD > LEFT/RIGHT > NONE
        /// </summary>
        void UpdateVehicleTurnSignal()
        {
            // HAZARD > LEFT, RIGHT > NONE
            if (hazardLightsSignal == EZ10Vehicle.TurnSignal.HAZARD)
                input = hazardLightsSignal;
            else if (turnIndicatorsSignal == EZ10Vehicle.TurnSignal.LEFT || turnIndicatorsSignal == EZ10Vehicle.TurnSignal.RIGHT)
                input = turnIndicatorsSignal;
            else
                input = EZ10Vehicle.TurnSignal.NONE;

            // input
            if (vehicle.SignalInput != input)
                vehicle.SignalInput = input;
        }

        void Start()
        {
            var qos = qosSettings.GetQoSProfile();

            turnIndicatorsCommandSubscriber
                = SimulatorROS2Node.CreateSubscription<autoware_auto_vehicle_msgs.msg.TurnIndicatorsCommand>(
                    turnIndicatorsCommandTopic, msg =>
                    {
                        turnIndicatorsSignal = EZ10VehicleROS2Utility.RosToUnityTurnSignal(msg);
                        UpdateVehicleTurnSignal();
                    }, qos);

            hazardLightsCommandSubscriber
                = SimulatorROS2Node.CreateSubscription<autoware_auto_vehicle_msgs.msg.HazardLightsCommand>(
                    hazardLightsCommandTopic, msg =>
                    {
                        hazardLightsSignal = EZ10VehicleROS2Utility.RosToUnityHazard(msg);
                        UpdateVehicleTurnSignal();
                    }, qos);

            ackermanControlCommandSubscriber
                = SimulatorROS2Node.CreateSubscription<autoware_auto_control_msgs.msg.AckermannControlCommand>(
                    ackermannControlCommandTopic, msg =>
                    {
                        // highest priority is EMERGENCY.
                        // If Emergency is true, ControlCommand is not used for vehicle acceleration input.
                        if (!isEmergency)
                            vehicle.AccelerationInput = msg.Longitudinal.Acceleration;

                        vehicle.SteerAngleInput = -(float)msg.Lateral.Steering_tire_angle * Mathf.Rad2Deg;
                    }, qos);

            gearCommandSubscriber
                = SimulatorROS2Node.CreateSubscription<autoware_auto_vehicle_msgs.msg.GearCommand>(
                    gearCommandTopic, msg =>
                    {
                        vehicle.AutomaticShiftInput = EZ10VehicleROS2Utility.RosToUnityShift(msg);
                    }, qos);

            vehicleEmergencyStampedSubscriber
                = SimulatorROS2Node.CreateSubscription<tier4_vehicle_msgs.msg.VehicleEmergencyStamped>(
                    vehicleEmergencyStampedTopic, msg =>
                    {
                        // highest priority is EMERGENCY.
                        // If emergency is true, emergencyDeceleration is applied to the vehicle's deceleration.
                        isEmergency = msg.Emergency;
                        if (isEmergency)
                            vehicle.AccelerationInput = emergencyDeceleration;
                    });
        }

        void OnDestroy()
        {
            SimulatorROS2Node.RemoveSubscription<autoware_auto_vehicle_msgs.msg.TurnIndicatorsCommand>(turnIndicatorsCommandSubscriber);
            SimulatorROS2Node.RemoveSubscription<autoware_auto_vehicle_msgs.msg.HazardLightsCommand>(hazardLightsCommandSubscriber);
            SimulatorROS2Node.RemoveSubscription<autoware_auto_control_msgs.msg.AckermannControlCommand>(ackermanControlCommandSubscriber);
            SimulatorROS2Node.RemoveSubscription<autoware_auto_vehicle_msgs.msg.GearCommand>(gearCommandSubscriber);
            SimulatorROS2Node.RemoveSubscription<tier4_vehicle_msgs.msg.VehicleEmergencyStamped>(vehicleEmergencyStampedSubscriber);
        }
    }
}