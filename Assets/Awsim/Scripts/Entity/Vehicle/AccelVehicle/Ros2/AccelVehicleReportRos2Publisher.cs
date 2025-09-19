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
    public class AccelVehicleReportRos2Publisher : MonoBehaviour
    {
        public string ControlModeReportTopic { get => _controlModeReportTopic; }
        public string GearReportTopic { get => _gearReportTopic; }
        public string SteeringReportTopic { get => _steeringReportTopic; }
        public string TurnIndicatorsReportTopic { get => _turnIndicatorsReportTopic; }
        public string HazardLightsReportTopic { get => _hazardLightsReportTopic; }
        public string VelocityReportTopic { get => _velocityReportTopic; }
        public string FrameId { get => _frameId; }
        public int PublishHz { get => _publishHz; }
        public QosSettings QosSettings { get => _qosSettings; }

        [SerializeField] AccelVehicle _vehicle;
        [SerializeField] AccelVehicleControlModeBasedInputter _controlModeBasedInputProvider;

        // Topics.
        [Header("Topic names")]
        [SerializeField] string _controlModeReportTopic = "/vehicle/status/control_mode";
        [SerializeField] string _gearReportTopic = "/vehicle/status/gear_status";
        [SerializeField] string _steeringReportTopic = "/vehicle/status/steering_status";
        [SerializeField] string _turnIndicatorsReportTopic = "/vehicle/status/turn_indicators_status";
        [SerializeField] string _hazardLightsReportTopic = "/vehicle/status/hazard_lights_status";
        [SerializeField] string _velocityReportTopic = "/vehicle/status/velocity_status";

        [Header("Publisher settings")]
        [SerializeField] string _frameId = "base_link";
        [SerializeField] int _publishHz = 30;
        [SerializeField] QosSettings _qosSettings;


        // Msgs.
        autoware_vehicle_msgs.msg.ControlModeReport _controlModeReportMsg;
        autoware_vehicle_msgs.msg.GearReport _gearReportMsg;
        autoware_vehicle_msgs.msg.SteeringReport _steeringReportMsg;
        autoware_vehicle_msgs.msg.TurnIndicatorsReport _turnIndicatorsReportMsg;
        autoware_vehicle_msgs.msg.HazardLightsReport _hazardLightsReportMsg;
        autoware_vehicle_msgs.msg.VelocityReport _velocityReportMsg;

        // Publishers.
        IPublisher<autoware_vehicle_msgs.msg.ControlModeReport> _controlModeReportPublisher;
        IPublisher<autoware_vehicle_msgs.msg.GearReport> _gearReportPublisher;
        IPublisher<autoware_vehicle_msgs.msg.SteeringReport> _steeringReportPublisher;
        IPublisher<autoware_vehicle_msgs.msg.TurnIndicatorsReport> _turnIndicatorsReportPublisher;
        IPublisher<autoware_vehicle_msgs.msg.HazardLightsReport> _hazardLightsReportPublisher;
        IPublisher<autoware_vehicle_msgs.msg.VelocityReport> _velocityReportPublisher;



        public void Initialize()
        {
            var qos = _qosSettings.GetQosProfile();

            // Create publishers.
            _controlModeReportPublisher = AwsimRos2Node.CreatePublisher<autoware_vehicle_msgs.msg.ControlModeReport>(_controlModeReportTopic, qos);
            _gearReportPublisher = AwsimRos2Node.CreatePublisher<autoware_vehicle_msgs.msg.GearReport>(_gearReportTopic, qos);
            _steeringReportPublisher = AwsimRos2Node.CreatePublisher<autoware_vehicle_msgs.msg.SteeringReport>(_steeringReportTopic, qos);
            _turnIndicatorsReportPublisher = AwsimRos2Node.CreatePublisher<autoware_vehicle_msgs.msg.TurnIndicatorsReport>(_turnIndicatorsReportTopic, qos);
            _hazardLightsReportPublisher = AwsimRos2Node.CreatePublisher<autoware_vehicle_msgs.msg.HazardLightsReport>(_hazardLightsReportTopic, qos);
            _velocityReportPublisher = AwsimRos2Node.CreatePublisher<autoware_vehicle_msgs.msg.VelocityReport>(_velocityReportTopic, qos);

            // Create msgs.
            _controlModeReportMsg = new autoware_vehicle_msgs.msg.ControlModeReport();
            _gearReportMsg = new autoware_vehicle_msgs.msg.GearReport();
            _steeringReportMsg = new autoware_vehicle_msgs.msg.SteeringReport();
            _turnIndicatorsReportMsg = new autoware_vehicle_msgs.msg.TurnIndicatorsReport();
            _hazardLightsReportMsg = new autoware_vehicle_msgs.msg.HazardLightsReport();
            _velocityReportMsg = new autoware_vehicle_msgs.msg.VelocityReport()
            {
                Header = new std_msgs.msg.Header()
                {
                    Frame_id = _frameId,
                }
            };

            // Start publishing.
            InvokeRepeating(nameof(Publish), 0f, 1.0f / _publishHz);
        }

        public void Initialize(string controlModeReportTopic,
                            string gearReportTopic,
                            string steeringReportTopic,
                            string turnIndicatorsReportTopic,
                            string hazardLightsReportTopic,
                            string velocityReportTopic,
                            string frameId,
                            int publishHz,
                            QosSettings qosSettings)
        {
            _controlModeReportTopic = controlModeReportTopic;
            _gearReportTopic = gearReportTopic;
            _steeringReportTopic = steeringReportTopic;
            _turnIndicatorsReportTopic = turnIndicatorsReportTopic;
            _hazardLightsReportTopic = hazardLightsReportTopic;
            _velocityReportTopic = velocityReportTopic;
            _frameId = frameId;
            _publishHz = publishHz;
            _qosSettings = qosSettings;

            Initialize();
        }

        void Publish()
        {
            // Update msgs.
            var controlMode = AccelVehicleRos2MsgConverter.UnityToRos2ControlMode(_controlModeBasedInputProvider.ControlMode);                         // Control mode.
            _controlModeReportMsg.Mode = controlMode;
            _gearReportMsg.Report = AccelVehicleRos2MsgConverter.UnityToRos2Gear(_vehicle.Gear);                                 // Gear.
            _steeringReportMsg.Steering_tire_angle = -1 * _vehicle.SteerTireAngle * Mathf.Deg2Rad;                          // Steering.
            _turnIndicatorsReportMsg.Report = AccelVehicleRos2MsgConverter.UnityToRos2TurnIndicators(_vehicle.TurnIndicators);   // Turn indicators.
            _hazardLightsReportMsg.Report = AccelVehicleRos2MsgConverter.UnityToRos2HazardLights(_vehicle.HazardLights);         // Hazard lights.
            var rosLinearVelocity = Ros2Utility.UnityToRos2Position(_vehicle.LocalVelocity);                                // Velocity reports.
            var rosAngularVelocity = Ros2Utility.UnityToRos2Position(_vehicle.AngularVelocity);
            _velocityReportMsg.Longitudinal_velocity = rosLinearVelocity.x;
            _velocityReportMsg.Lateral_velocity = rosLinearVelocity.y;
            _velocityReportMsg.Heading_rate = rosAngularVelocity.z;

            // Update Stamp
            // NOTE: It may be better to set the same time value for all of them. If so, create a new API in AwsimRos2Node?
            AwsimRos2Node.UpdateROSClockTime(_controlModeReportMsg.Stamp);
            AwsimRos2Node.UpdateROSClockTime(_gearReportMsg.Stamp);
            AwsimRos2Node.UpdateROSClockTime(_steeringReportMsg.Stamp);
            AwsimRos2Node.UpdateROSClockTime(_turnIndicatorsReportMsg.Stamp);
            AwsimRos2Node.UpdateROSClockTime(_hazardLightsReportMsg.Stamp);
            var velocityReportMsgHeader = _velocityReportMsg as MessageWithHeader;
            AwsimRos2Node.UpdateROSTimestamp(ref velocityReportMsgHeader);

            // Publish.
            _controlModeReportPublisher.Publish(_controlModeReportMsg);
            _gearReportPublisher.Publish(_gearReportMsg);
            _steeringReportPublisher.Publish(_steeringReportMsg);
            _turnIndicatorsReportPublisher.Publish(_turnIndicatorsReportMsg);
            _hazardLightsReportPublisher.Publish(_hazardLightsReportMsg);
            _velocityReportPublisher.Publish(_velocityReportMsg);
        }

        void OnDestroy()
        {
            AwsimRos2Node.RemovePublisher<autoware_vehicle_msgs.msg.ControlModeReport>(_controlModeReportPublisher);
            AwsimRos2Node.RemovePublisher<autoware_vehicle_msgs.msg.GearReport>(_gearReportPublisher);
            AwsimRos2Node.RemovePublisher<autoware_vehicle_msgs.msg.SteeringReport>(_steeringReportPublisher);
            AwsimRos2Node.RemovePublisher<autoware_vehicle_msgs.msg.TurnIndicatorsReport>(_turnIndicatorsReportPublisher);
            AwsimRos2Node.RemovePublisher<autoware_vehicle_msgs.msg.HazardLightsReport>(_hazardLightsReportPublisher);
            AwsimRos2Node.RemovePublisher<autoware_vehicle_msgs.msg.VelocityReport>(_velocityReportPublisher);
        }
    }
}