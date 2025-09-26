// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using UnityEngine;
using ROS2;
using Awsim.Common;

namespace Awsim.Entity
{
    public class AccelVehicleRos2Input : MonoBehaviour, IAccelVehicleInput
    {
        public string Name => "Ros2";

        public float AccelerationInput { get; private set; }

        public float SteerAngleInput { get; private set; }

        public Gear GearInput { get; private set; }

        public TurnIndicators TurnIndicatorsInput { get; private set; }

        public HazardLights HazardLightsInput { get; private set; }

        public bool SwitchAutonomous { get; private set; } = false;

        [SerializeField] string _turnIndicatorsCommandTopic = "/control/command/turn_indicators_cmd";
        [SerializeField] string _hazardLightsCommandTopic = "/control/command/hazard_lights_cmd";
        [SerializeField] string _ackermannControlCommandTopic = "/control/command/control_cmd";
        [SerializeField] string _gearCommandTopic = "/control/command/gear_cmd";
        [SerializeField] string _vehicleEmergencyStampedTopic = "/control/command/emergency_cmd";
        [SerializeField]
        QosSettings _qosSettings = new QosSettings(ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
                                                   DurabilityPolicy.QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
                                                   HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
                                                   1);

        // Subscribers.
        ISubscription<autoware_vehicle_msgs.msg.TurnIndicatorsCommand> _turnIndicatorsCommandSubscriber;
        ISubscription<autoware_vehicle_msgs.msg.HazardLightsCommand> _hazardLightsCommandSubscriber;
        ISubscription<autoware_control_msgs.msg.Control> _ackermanControlCommandSubscriber;
        ISubscription<autoware_vehicle_msgs.msg.GearCommand> _gearCommandSubscriber;
        ISubscription<tier4_vehicle_msgs.msg.VehicleEmergencyStamped> _vehicleEmergencyStampedSubscriber;

        float _accelerationInput = 0;
        float _steerAngleInput = 0;
        Gear _gearInput = Gear.Parking;
        TurnIndicators _turnIndicatorsInput = TurnIndicators.None;
        HazardLights _hazardLightsInput = HazardLights.Disable;

        public void Initialize()
        {
            var qos = _qosSettings.GetQosProfile();

            _turnIndicatorsCommandSubscriber
                = AwsimRos2Node.CreateSubscription<autoware_vehicle_msgs.msg.TurnIndicatorsCommand>(_turnIndicatorsCommandTopic, msg =>
                {
                    _turnIndicatorsInput = AccelVehicleRos2MsgConverter.Ros2ToUnityTurnIndicators(msg);
                }, qos);

            _hazardLightsCommandSubscriber
                = AwsimRos2Node.CreateSubscription<autoware_vehicle_msgs.msg.HazardLightsCommand>(_hazardLightsCommandTopic, msg =>
                {
                    _hazardLightsInput = AccelVehicleRos2MsgConverter.Ros2ToUnityHazardLights(msg);
                }, qos);

            _ackermanControlCommandSubscriber
                = AwsimRos2Node.CreateSubscription<autoware_control_msgs.msg.Control>(_ackermannControlCommandTopic, msg =>
                {
                    _accelerationInput = msg.Longitudinal.Acceleration;
                    _steerAngleInput = -(float)msg.Lateral.Steering_tire_angle * Mathf.Rad2Deg;
                }, qos);

            _gearCommandSubscriber
                = AwsimRos2Node.CreateSubscription<autoware_vehicle_msgs.msg.GearCommand>(_gearCommandTopic, msg =>
                {
                    _gearInput = AccelVehicleRos2MsgConverter.Ros2ToUnityGear(msg);
                }, qos);

            _vehicleEmergencyStampedSubscriber
                = AwsimRos2Node.CreateSubscription<tier4_vehicle_msgs.msg.VehicleEmergencyStamped>(_vehicleEmergencyStampedTopic, msg =>
                {

                }, qos);
        }

        public bool UpdateInputs()
        {
            AccelerationInput = _accelerationInput;
            SteerAngleInput = _steerAngleInput;
            GearInput = _gearInput;
            TurnIndicatorsInput = _turnIndicatorsInput;
            HazardLightsInput = _hazardLightsInput;

            return false;
        }

        void OnDestroy()
        {
            // TODO: remove subscription.
        }
    }
}