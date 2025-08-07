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

namespace Awsim.Entity
{
    /// <summary>
    /// Autoware's ControlMode.
    /// </summary>
    /// <see href="https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/ControlModeReport.msg"/>
    public enum ControlMode
    {
        // TODO: Support for commented-out control modes

        //NO_COMMAND                      = 0,
        Autonomous = 1,
        //AUTONOMOUS_STEER_ONLY           = 2,
        //AUTONOMOUS_VELOCITY_ONLY        = 3,
        Manual = 4,
        //DISENGAGED                      = 5,
        //NOT_READY                       = 6
    }
}