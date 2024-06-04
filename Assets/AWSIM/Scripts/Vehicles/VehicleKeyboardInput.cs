using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// Controlling vehicle input via keyboard.
    /// </summary>
    public class VehicleKeyboardInput : VehicleInputBase
    {
        public float MaxAcceleraion = 1.5f;
        float MaxSteerAngle = 0;
        [SerializeField] Vehicle vehicle;

        void OnEnable()
        {
            MaxSteerAngle = vehicle.MaxSteerAngleInput;
        }

        public override void OnUpdate(VehicleControlMode currentControlMode)
        {
            Overriden = false;

            // Get steering and acceleraion input.
            var horizontal = Input.GetAxis("Horizontal");
            var vertical = Input.GetAxis("Vertical");
            AccelerationInput = MaxAcceleraion * vertical;
            SteeringInput = MaxSteerAngle * horizontal;

            // Get gear input.
            if (Input.GetKey(KeyCode.D))
                ShiftInput = Vehicle.Shift.DRIVE;
            else if (Input.GetKey(KeyCode.P))
                ShiftInput = Vehicle.Shift.PARKING;
            else if (Input.GetKey(KeyCode.R))
                ShiftInput = Vehicle.Shift.REVERSE;
            else if (Input.GetKey(KeyCode.N))
                ShiftInput = Vehicle.Shift.NEUTRAL;
            else
                ShiftInput = vehicle.AutomaticShift;    // No new input.

            // Get turn signal input.
            if (Input.GetKey(KeyCode.Alpha1))
                TurnSignalInput = Vehicle.TurnSignal.LEFT;
            else if (Input.GetKey(KeyCode.Alpha2))
                TurnSignalInput = Vehicle.TurnSignal.RIGHT;
            else if (Input.GetKey(KeyCode.Alpha3))
                TurnSignalInput = Vehicle.TurnSignal.HAZARD;
            else if (Input.GetKey(KeyCode.Alpha4))
                TurnSignalInput = Vehicle.TurnSignal.NONE;
            else
                TurnSignalInput = vehicle.Signal;       // No new input.

            // Input override considerations.
            if (currentControlMode == VehicleControlMode.AUTONOMOUS)
            {
                // Override the vehicle's control mode when a steering input or acceleration input is given.
                if (Mathf.Abs(horizontal) > 0 || Mathf.Abs(vertical) > 0)
                {
                    Overriden = true;
                    NewControlMode = VehicleControlMode.MANUAL;
                }

                // TODO: Implement switches to other overrides.
            }
        }
    }
}