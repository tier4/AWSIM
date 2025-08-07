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
    public class AccelVehicleRos2MsgConverter
    {
        /// <summary>
        /// Convert the autoware's ros2 gear msg to awsim's unity gear.
        /// </summary>
        /// <param name="gearCommand"></param>
        /// <returns>Converted gear.</returns>
        public static Gear Ros2ToUnityGear(autoware_vehicle_msgs.msg.GearCommand gearCommand)
        {
            // NONE, PARKING to PARKING.
            if (gearCommand.Command == autoware_vehicle_msgs.msg.GearCommand.NONE ||
                gearCommand.Command == autoware_vehicle_msgs.msg.GearCommand.PARK)
                return Gear.Parking;
            // REVERSE to REVERSE.
            else if (gearCommand.Command == autoware_vehicle_msgs.msg.GearCommand.REVERSE)
                return Gear.Reverse;
            // NEUTEAL to NEUTEAL.
            else if (gearCommand.Command == autoware_vehicle_msgs.msg.GearCommand.NEUTRAL)
                return Gear.Neutral;
            // DRIVE, LOW to DRIVE.
            else if (gearCommand.Command == autoware_vehicle_msgs.msg.GearCommand.DRIVE ||
                     gearCommand.Command == autoware_vehicle_msgs.msg.GearCommand.LOW)
                return Gear.Drive;
            else
                return Gear.Parking;
        }

        /// <summary>
        /// Convert the awsim's unity gear to autoware's ros2 gear msg.
        /// </summary>
        /// <param name="gear"></param>
        /// <returns>Converted gear.</returns>
        public static byte UnityToRos2Gear(Gear gear)
        {
            if (gear == Gear.Parking)
                return autoware_vehicle_msgs.msg.GearReport.PARK;
            else if (gear == Gear.Reverse)
                return autoware_vehicle_msgs.msg.GearReport.REVERSE;
            else if (gear == Gear.Neutral)
                return autoware_vehicle_msgs.msg.GearReport.NEUTRAL;
            else if (gear == Gear.Drive)
                return autoware_vehicle_msgs.msg.GearReport.DRIVE;
            else
                return autoware_vehicle_msgs.msg.GearReport.PARK;
        }

        /// <summary>
        /// Convert the autoware's ros2 turn indicators msg to awsim's unity turn indicators.
        /// </summary>
        /// <param name="turnIndicatorsCommand"></param>
        /// <returns>Converted turn indicators.</returns>
        public static TurnIndicators Ros2ToUnityTurnIndicators(autoware_vehicle_msgs.msg.TurnIndicatorsCommand turnIndicatorsCommand)
        {
            if (turnIndicatorsCommand.Command == autoware_vehicle_msgs.msg.TurnIndicatorsCommand.DISABLE)
                return TurnIndicators.None;
            else if (turnIndicatorsCommand.Command == autoware_vehicle_msgs.msg.TurnIndicatorsCommand.ENABLE_LEFT)
                return TurnIndicators.Left;
            else if (turnIndicatorsCommand.Command == autoware_vehicle_msgs.msg.TurnIndicatorsCommand.ENABLE_RIGHT)
                return TurnIndicators.Right;
            else
                return TurnIndicators.None;
        }

        /// <summary>
        /// Convert the awsim's unity turn indicators to autoware's ros2 turn indicators msg.
        /// </summary>
        /// <param name="turnIndicators"></param>
        /// <returns>Converted turn indicators.</returns>
        public static byte UnityToRos2TurnIndicators(TurnIndicators turnIndicators)
        {
            if (turnIndicators == TurnIndicators.None)
                return autoware_vehicle_msgs.msg.TurnIndicatorsReport.DISABLE;
            else if (turnIndicators == TurnIndicators.Left)
                return autoware_vehicle_msgs.msg.TurnIndicatorsReport.ENABLE_LEFT;
            else if (turnIndicators == TurnIndicators.Right)
                return autoware_vehicle_msgs.msg.TurnIndicatorsReport.ENABLE_RIGHT;
            else
                return autoware_vehicle_msgs.msg.TurnIndicatorsReport.DISABLE;
        }

        /// <summary>
        /// Convert the autoware's ros2 hazard lights msg to awsim's unity turn indicators.
        /// </summary>
        /// <param name="turnSignalMsg"></param>
        /// <returns>Converted turn indicators.</returns>
        public static HazardLights Ros2ToUnityHazardLights(autoware_vehicle_msgs.msg.HazardLightsCommand hazardLightsCommand)
        {
            if (hazardLightsCommand.Command == autoware_vehicle_msgs.msg.HazardLightsCommand.ENABLE)
                return HazardLights.Enable;
            else if (hazardLightsCommand.Command == autoware_vehicle_msgs.msg.HazardLightsCommand.DISABLE)
                return HazardLights.Disable;
            else
                return HazardLights.Disable;
        }

        /// <summary>
        /// Convert the awsim's unity turn indicators to autoware's ros2 hazard lights.
        /// </summary>
        /// <param name="hazardLights"></param>
        /// <returns>Converted turn signal.</returns>
        public static byte UnityToRos2HazardLights(HazardLights hazardLights)
        {
            if (hazardLights == HazardLights.Enable)
                return autoware_vehicle_msgs.msg.HazardLightsReport.ENABLE;
            else if (hazardLights == HazardLights.Disable)
                return autoware_vehicle_msgs.msg.HazardLightsReport.DISABLE;
            else
                return autoware_vehicle_msgs.msg.HazardLightsReport.DISABLE;
        }

        /// <summary>
        /// Convert the autoware's ros2 control mode to awsim's unity control mode.
        /// </summary>
        /// <param name="controlModeRequest"></param>
        /// <returns>Converted control mode.</returns>
        public static ControlMode Ros2ToUnityControlMode(autoware_vehicle_msgs.srv.ControlModeCommand_Request controlModeRequest)
        {
            if (controlModeRequest.Mode == autoware_vehicle_msgs.msg.ControlModeReport.AUTONOMOUS)
                return ControlMode.Autonomous;
            else if (controlModeRequest.Mode == autoware_vehicle_msgs.msg.ControlModeReport.MANUAL)
                return ControlMode.Manual;
            else
                return ControlMode.Manual;
        }

        /// <summary>
        /// Convert the awsim's unity control mode to autoware's ros2 control mode.
        /// </summary>
        /// <param name="controlMode"></param>
        /// <returns>Converted control mode.</returns>
        public static byte UnityToRos2ControlMode(ControlMode controlMode)
        {
            if (controlMode == ControlMode.Autonomous)
                return autoware_vehicle_msgs.msg.ControlModeReport.AUTONOMOUS;
            else if (controlMode == ControlMode.Manual)
                return autoware_vehicle_msgs.msg.ControlModeReport.MANUAL;
            else
                return autoware_vehicle_msgs.msg.ControlModeReport.AUTONOMOUS;
        }
    }
}