using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;
using AWSIM;

namespace AWSIM
{
    /// <summary>
    /// ros2 service communication about ControlModeCommand and switches the ControlMode of the Vehicle.
    /// i.e. "ros2 service call input/control_mode_request autoware_vehicle_msgs/srv/ControlModeCommand "mode: 1"
    /// </summary>
    public class ControlModeSrvServer : MonoBehaviour
    {
        [SerializeField] string serviceName = "input/control_mode_request";

        IService<autoware_vehicle_msgs.srv.ControlModeCommand_Request, autoware_vehicle_msgs.srv.ControlModeCommand_Response> service;

        [SerializeField] VehicleOverrideInputManager vehicleOverrideInputManager;

        public autoware_vehicle_msgs.srv.ControlModeCommand_Response ChangeControlMode(autoware_vehicle_msgs.srv.ControlModeCommand_Request request)
        {
            autoware_vehicle_msgs.srv.ControlModeCommand_Response response = new autoware_vehicle_msgs.srv.ControlModeCommand_Response();
            if (request.Mode == autoware_vehicle_msgs.srv.ControlModeCommand_Request.AUTONOMOUS)
            {
                vehicleOverrideInputManager.ControlMode = VehicleControlMode.AUTONOMOUS;
            }
            // else if (request.Mode == autoware_vehicle_msgs.srv.ControlModeCommand_Request.AUTONOMOUS_STEER_ONLY)
            // {

            // }
            // else if (request.Mode == autoware_vehicle_msgs.srv.ControlModeCommand_Request.AUTONOMOUS_VELOCITY_ONLY)
            // {

            // }
            else if (request.Mode == autoware_vehicle_msgs.srv.ControlModeCommand_Request.MANUAL)
            {
                vehicleOverrideInputManager.ControlMode = VehicleControlMode.MANUAL;
            }
            // else if (request.Mode == autoware_vehicle_msgs.srv.ControlModeCommand_Request.NO_COMMAND)
            // {

            // }

            response.Success = true;
            return response;
        }

        void Start()
        {
            service = SimulatorROS2Node.CreateService<autoware_vehicle_msgs.srv.ControlModeCommand_Request, autoware_vehicle_msgs.srv.ControlModeCommand_Response>
            (serviceName, ChangeControlMode);
        }
    }
}