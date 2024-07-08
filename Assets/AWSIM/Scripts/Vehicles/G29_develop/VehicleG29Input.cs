using System.Collections;
using System.Collections.Generic;
using tier4_vehicle_msgs.msg;
using UnityEngine;
using UnityEngine.InputSystem;

namespace AWSIM
{
    public class VehicleG29Input : VehicleInputBase
    {
        public float MaxAcceleration = 1.5f;

        public override void OnUpdate(VehicleControlMode currentControlMode)
        {
            Overridden = true;
            NewControlMode = VehicleControlMode.MANUAL;
        }

        public void OnThrottle(InputAction.CallbackContext context)
        {
            var throttle = context.ReadValue<float>();
            AccelerationInput = MaxAcceleration * throttle;
        }

        public void OnBrake(InputAction.CallbackContext context)
        {
            var brake = context.ReadValue<float>();
            AccelerationInput = MaxAcceleration * -brake;
        }

        public void OnDriveGear(InputAction.CallbackContext context)
        {
            if (ShiftInput != Vehicle.Shift.DRIVE)
                ShiftInput = Vehicle.Shift.DRIVE;
        }

        public void OnReverseGear(InputAction.CallbackContext context)
        {
            if (ShiftInput != Vehicle.Shift.REVERSE)
                ShiftInput = Vehicle.Shift.REVERSE;
        }

        public void OnNeutralGear(InputAction.CallbackContext context)
        {
            if (ShiftInput != Vehicle.Shift.NEUTRAL)
                ShiftInput = Vehicle.Shift.NEUTRAL;
        }

        public void OnParkingGear(InputAction.CallbackContext context)
        {
            if (ShiftInput != Vehicle.Shift.PARKING)
                ShiftInput = Vehicle.Shift.PARKING;
        }

        public void OnTurnSignalNone(InputAction.CallbackContext context)
        {
            if (TurnSignalInput != Vehicle.TurnSignal.NONE)
                TurnSignalInput = Vehicle.TurnSignal.NONE;
        }

        public void OnTurnSignalLeft(InputAction.CallbackContext context)
        {
            if (TurnSignalInput != Vehicle.TurnSignal.LEFT)
                TurnSignalInput = Vehicle.TurnSignal.LEFT;
        }

        public void OnTurnSignalRight(InputAction.CallbackContext context)
        {
            if (TurnSignalInput != Vehicle.TurnSignal.RIGHT)
                TurnSignalInput = Vehicle.TurnSignal.RIGHT;
        }

        public void OnTurnSignalHazard(InputAction.CallbackContext context)
        {
            if (TurnSignalInput != Vehicle.TurnSignal.HAZARD)
                TurnSignalInput = Vehicle.TurnSignal.HAZARD;
        }

        public void OnSteering(InputAction.CallbackContext context)
        {
            var steer = context.ReadValue<float>();
            SteeringInput = steer * 35;

            Debug.Log(Time.time);
            //Debug.Log(SteeringInput);
        }
    }
}