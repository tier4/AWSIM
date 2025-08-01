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
    /// <summary>
    /// ros2 service communication about ControlModeCommand and switches the ControlMode of the Vehicle.
    /// i.e. "ros2 service call input/control_mode_request autoware_vehicle_msgs/srv/ControlModeCommand "mode: 1"
    /// </summary>
    public class AccelVehicleControlModeSrvServer : MonoBehaviour
    {
        [SerializeField] string serviceName = "input/control_mode_request";

        IService<autoware_vehicle_msgs.srv.ControlModeCommand_Request, autoware_vehicle_msgs.srv.ControlModeCommand_Response> service;

        [SerializeField] AccelVehicleControlModeBasedInputter _controlModeBasedInputter;

        public autoware_vehicle_msgs.srv.ControlModeCommand_Response ChangeControlMode(autoware_vehicle_msgs.srv.ControlModeCommand_Request request)
        {
            autoware_vehicle_msgs.srv.ControlModeCommand_Response response = new autoware_vehicle_msgs.srv.ControlModeCommand_Response();
            if (request.Mode == autoware_vehicle_msgs.srv.ControlModeCommand_Request.AUTONOMOUS)
            {
                _controlModeBasedInputter.ControlMode = ControlMode.Autonomous;
            }
            // else if (request.Mode == autoware_vehicle_msgs.srv.ControlModeCommand_Request.AUTONOMOUS_STEER_ONLY)
            // {

            // }
            // else if (request.Mode == autoware_vehicle_msgs.srv.ControlModeCommand_Request.AUTONOMOUS_VELOCITY_ONLY)
            // {

            // }
            else if (request.Mode == autoware_vehicle_msgs.srv.ControlModeCommand_Request.MANUAL)
            {
                _controlModeBasedInputter.ControlMode = ControlMode.Manual;
            }
            // else if (request.Mode == autoware_vehicle_msgs.srv.ControlModeCommand_Request.NO_COMMAND)
            // {

            // }

            response.Success = true;
            return response;
        }

        public void Initialize()
        {
            service = AwsimRos2Node.CreateService<autoware_vehicle_msgs.srv.ControlModeCommand_Request, autoware_vehicle_msgs.srv.ControlModeCommand_Response>
            (serviceName, ChangeControlMode);
        }
    }
}