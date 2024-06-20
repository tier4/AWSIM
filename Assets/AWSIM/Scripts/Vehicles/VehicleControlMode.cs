using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// C# enum for Autoware's ControlMode. (No dependency on ROS2)
    /// VehicleROS2Utility.UnityToRosControlMode() can be used to convert C# enum to ros msg.
    /// <see href="https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/ControlModeReport.msg"/>
    /// </summary>
    public enum VehicleControlMode
    {
        // NO_COMMAND = 0,
        AUTONOMOUS = 1,
        // AUTONOMOUS_STEER_ONLY = 2,
        // AUTONOMOUS_VELOCITY_ONLY = 3,
        MANUAL = 4,
        // DISENGAGED = 5,
        // NOT_READY = 6
    }
}