using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// Vehicle ROS2 utility static class.
    /// </summary>
    public static class VehicleROS2Utility
    {
        /// <summary>
        /// Convert the ROS msg shift to Vehicle.Shift.
        /// </summary>
        /// <param name="shiftMsg"></param>
        /// <returns>Converted shift.</returns>
        public static Vehicle.Shift RosToUnityShift(autoware_auto_vehicle_msgs.msg.GearCommand gearCommand)
        {
            // NONE, PARKING to PARKING.
            if (gearCommand.Command == autoware_auto_vehicle_msgs.msg.GearReport.NONE ||
                gearCommand.Command == autoware_auto_vehicle_msgs.msg.GearReport.PARK)
                return Vehicle.Shift.PARKING;
            // REVERSE to REVERSE.
            else if (gearCommand.Command == autoware_auto_vehicle_msgs.msg.GearReport.REVERSE)
                return Vehicle.Shift.REVERSE;
            // NEUTEAL to NEUTEAL.
            else if (gearCommand.Command == autoware_auto_vehicle_msgs.msg.GearReport.NEUTRAL)
                return Vehicle.Shift.NEUTRAL;
            // DRIVE, LOW to DRIVE.
            else if (gearCommand.Command == autoware_auto_vehicle_msgs.msg.GearReport.DRIVE ||
                     gearCommand.Command == autoware_auto_vehicle_msgs.msg.GearReport.LOW)
                return Vehicle.Shift.DRIVE;
            else
                return Vehicle.Shift.PARKING;
        }

        /// <summary>
        /// Convert the Vehicle.Shift to ROS msg shift.
        /// </summary>
        /// <param name="shift"></param>
        /// <returns>Converted shift.</returns>
        public static byte UnityToRosShift(Vehicle.Shift shift)
        {
            if (shift == Vehicle.Shift.PARKING)
                return autoware_auto_vehicle_msgs.msg.GearReport.PARK;
            else if (shift == Vehicle.Shift.REVERSE)
                return autoware_auto_vehicle_msgs.msg.GearReport.REVERSE;
            else if (shift == Vehicle.Shift.NEUTRAL)
                return autoware_auto_vehicle_msgs.msg.GearReport.NEUTRAL;
            else if (shift == Vehicle.Shift.DRIVE)
                return autoware_auto_vehicle_msgs.msg.GearReport.DRIVE;
            else
                return autoware_auto_vehicle_msgs.msg.GearReport.PARK;
        }

        /// <summary>
        /// Convert the ROS msg TurnIndicators to Vehicle.TurnSignal.
        /// </summary>
        /// <param name="turnSignalMsg"></param>
        /// <returns>Converted turn signal.</returns>
        public static Vehicle.TurnSignal RosToUnityTurnSignal(autoware_auto_vehicle_msgs.msg.TurnIndicatorsCommand turnIndicatorsCommand)
        {
            if (turnIndicatorsCommand.Command == autoware_auto_vehicle_msgs.msg.TurnIndicatorsCommand.DISABLE)
                return Vehicle.TurnSignal.NONE;
            else if (turnIndicatorsCommand.Command == autoware_auto_vehicle_msgs.msg.TurnIndicatorsCommand.ENABLE_LEFT)
                return Vehicle.TurnSignal.LEFT;
            else if (turnIndicatorsCommand.Command == autoware_auto_vehicle_msgs.msg.TurnIndicatorsCommand.ENABLE_RIGHT)
                return Vehicle.TurnSignal.RIGHT;
            else
                return Vehicle.TurnSignal.NONE;
        }

        /// <summary>
        /// Convert the Vehicle.TurnSignal to ROS msg turnSignal.
        /// </summary>
        /// <param name="turnSignal"></param>
        /// <returns>Converted turn signal.</returns>
        public static byte UnityToRosTurnSignal(Vehicle.TurnSignal turnSignal)
        {
            if (turnSignal == Vehicle.TurnSignal.NONE)
                return autoware_auto_vehicle_msgs.msg.TurnIndicatorsReport.DISABLE;
            else if (turnSignal == Vehicle.TurnSignal.LEFT)
                return autoware_auto_vehicle_msgs.msg.TurnIndicatorsReport.ENABLE_LEFT;
            else if (turnSignal == Vehicle.TurnSignal.RIGHT)
                return autoware_auto_vehicle_msgs.msg.TurnIndicatorsReport.ENABLE_RIGHT;
            else
                return autoware_auto_vehicle_msgs.msg.TurnIndicatorsReport.DISABLE;
        }

        public static Vehicle.TurnSignal RosToUnityHazard(autoware_auto_vehicle_msgs.msg.HazardLightsCommand hazardLightsCommand)
        {
            if (hazardLightsCommand.Command == autoware_auto_vehicle_msgs.msg.HazardLightsCommand.ENABLE)
                return Vehicle.TurnSignal.HAZARD;
            else if (hazardLightsCommand.Command == autoware_auto_vehicle_msgs.msg.HazardLightsCommand.DISABLE)
                return Vehicle.TurnSignal.NONE;
            else
                return Vehicle.TurnSignal.NONE;
        }

        public static byte UnityToRosHazard(Vehicle.TurnSignal turnSignal)
        {
            if (turnSignal == Vehicle.TurnSignal.HAZARD)
                return autoware_auto_vehicle_msgs.msg.HazardLightsReport.ENABLE;
            else if (turnSignal == Vehicle.TurnSignal.NONE)
                return autoware_auto_vehicle_msgs.msg.HazardLightsReport.DISABLE;
            else
                return autoware_auto_vehicle_msgs.msg.HazardLightsReport.DISABLE;
        }
    }
}