using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// EZ10Vehicle ROS2 utility static class.
    /// </summary>
    public static class EZ10VehicleROS2Utility
    {
        /// <summary>
        /// Convert the ROS msg shift to EZ10Vehicle.Shift.
        /// </summary>
        /// <param name="shiftMsg"></param>
        /// <returns>Converted shift.</returns>
        public static EZ10Vehicle.Shift RosToUnityShift(autoware_auto_vehicle_msgs.msg.GearCommand gearCommand)
        {
            // NONE, PARKING to PARKING.
            if (gearCommand.Command == autoware_auto_vehicle_msgs.msg.GearReport.NONE ||
                gearCommand.Command == autoware_auto_vehicle_msgs.msg.GearReport.PARK)
                return EZ10Vehicle.Shift.PARKING;
            // REVERSE to REVERSE.
            else if (gearCommand.Command == autoware_auto_vehicle_msgs.msg.GearReport.REVERSE)
                return EZ10Vehicle.Shift.REVERSE;
            // NEUTEAL to NEUTEAL.
            else if (gearCommand.Command == autoware_auto_vehicle_msgs.msg.GearReport.NEUTRAL)
                return EZ10Vehicle.Shift.NEUTRAL;
            // DRIVE, LOW to DRIVE.
            else if (gearCommand.Command == autoware_auto_vehicle_msgs.msg.GearReport.DRIVE ||
                     gearCommand.Command == autoware_auto_vehicle_msgs.msg.GearReport.LOW)
                return EZ10Vehicle.Shift.DRIVE;
            else
                return EZ10Vehicle.Shift.PARKING;
        }

        /// <summary>
        /// Convert the EZ10Vehicle.Shift to ROS msg shift.
        /// </summary>
        /// <param name="shift"></param>
        /// <returns>Converted shift.</returns>
        public static byte UnityToRosShift(EZ10Vehicle.Shift shift)
        {
            if (shift == EZ10Vehicle.Shift.PARKING)
                return autoware_auto_vehicle_msgs.msg.GearReport.PARK;
            else if (shift == EZ10Vehicle.Shift.REVERSE)
                return autoware_auto_vehicle_msgs.msg.GearReport.REVERSE;
            else if (shift == EZ10Vehicle.Shift.NEUTRAL)
                return autoware_auto_vehicle_msgs.msg.GearReport.NEUTRAL;
            else if (shift == EZ10Vehicle.Shift.DRIVE)
                return autoware_auto_vehicle_msgs.msg.GearReport.DRIVE;
            else
                return autoware_auto_vehicle_msgs.msg.GearReport.PARK;
        }

        /// <summary>
        /// Convert the ROS msg TurnIndicators to EZ10Vehicle.TurnSignal.
        /// </summary>
        /// <param name="turnSignalMsg"></param>
        /// <returns>Converted turn signal.</returns>
        public static EZ10Vehicle.TurnSignal RosToUnityTurnSignal(autoware_auto_vehicle_msgs.msg.TurnIndicatorsCommand turnIndicatorsCommand)
        {
            if (turnIndicatorsCommand.Command == autoware_auto_vehicle_msgs.msg.TurnIndicatorsCommand.DISABLE)
                return EZ10Vehicle.TurnSignal.NONE;
            else if (turnIndicatorsCommand.Command == autoware_auto_vehicle_msgs.msg.TurnIndicatorsCommand.ENABLE_LEFT)
                return EZ10Vehicle.TurnSignal.LEFT;
            else if (turnIndicatorsCommand.Command == autoware_auto_vehicle_msgs.msg.TurnIndicatorsCommand.ENABLE_RIGHT)
                return EZ10Vehicle.TurnSignal.RIGHT;
            else
                return EZ10Vehicle.TurnSignal.NONE;
        }

        /// <summary>
        /// Convert the EZ10Vehicle.TurnSignal to ROS msg turnSignal.
        /// </summary>
        /// <param name="turnSignal"></param>
        /// <returns>Converted turn signal.</returns>
        public static byte UnityToRosTurnSignal(EZ10Vehicle.TurnSignal turnSignal)
        {
            if (turnSignal == EZ10Vehicle.TurnSignal.NONE)
                return autoware_auto_vehicle_msgs.msg.TurnIndicatorsReport.DISABLE;
            else if (turnSignal == EZ10Vehicle.TurnSignal.LEFT)
                return autoware_auto_vehicle_msgs.msg.TurnIndicatorsReport.ENABLE_LEFT;
            else if (turnSignal == EZ10Vehicle.TurnSignal.RIGHT)
                return autoware_auto_vehicle_msgs.msg.TurnIndicatorsReport.ENABLE_RIGHT;
            else
                return autoware_auto_vehicle_msgs.msg.TurnIndicatorsReport.DISABLE;
        }

        public static EZ10Vehicle.TurnSignal RosToUnityHazard(autoware_auto_vehicle_msgs.msg.HazardLightsCommand hazardLightsCommand)
        {
            if (hazardLightsCommand.Command == autoware_auto_vehicle_msgs.msg.HazardLightsCommand.ENABLE)
                return EZ10Vehicle.TurnSignal.HAZARD;
            else if (hazardLightsCommand.Command == autoware_auto_vehicle_msgs.msg.HazardLightsCommand.DISABLE)
                return EZ10Vehicle.TurnSignal.NONE;
            else
                return EZ10Vehicle.TurnSignal.NONE;
        }

        public static byte UnityToRosHazard(EZ10Vehicle.TurnSignal turnSignal)
        {
            if (turnSignal == EZ10Vehicle.TurnSignal.HAZARD)
                return autoware_auto_vehicle_msgs.msg.HazardLightsReport.ENABLE;
            else if (turnSignal == EZ10Vehicle.TurnSignal.NONE)
                return autoware_auto_vehicle_msgs.msg.HazardLightsReport.DISABLE;
            else
                return autoware_auto_vehicle_msgs.msg.HazardLightsReport.DISABLE;
        }
    }
}