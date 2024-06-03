using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    public class VehicleKeyboardInput : VehicleInputBase
    {
        [SerializeField] Vehicle vehicle;

        public float MaxAcceleraion = 1.5f;
        public float MaxSteerAngle = 35f;

        public override void OnUpdate(VehicleControlMode currentControlMode)
        {
            Overriden = false;

            var horizontal = Input.GetAxis("Horizontal");
            var vertical = Input.GetAxis("Vertical");

            AccelerationInput = MaxAcceleraion * vertical;
            SteeringInput = MaxSteerAngle * horizontal;

            // set gear
            if (Input.GetKey(KeyCode.D))
                ShiftInput = Vehicle.Shift.DRIVE;
            else if (Input.GetKey(KeyCode.P))
                ShiftInput = Vehicle.Shift.PARKING;
            else if (Input.GetKey(KeyCode.R))
                ShiftInput = Vehicle.Shift.REVERSE;
            else if (Input.GetKey(KeyCode.N))
                ShiftInput = Vehicle.Shift.NEUTRAL;
            else
                ShiftInput = vehicle.AutomaticShift;

            // set turn signal
            if (Input.GetKey(KeyCode.Alpha1))
                TurnSignalInput = Vehicle.TurnSignal.LEFT;
            else if (Input.GetKey(KeyCode.Alpha2))
                TurnSignalInput = Vehicle.TurnSignal.RIGHT;
            else if (Input.GetKey(KeyCode.Alpha3))
                TurnSignalInput = Vehicle.TurnSignal.HAZARD;
            else if (Input.GetKey(KeyCode.Alpha4))
                TurnSignalInput = Vehicle.TurnSignal.NONE;
            else
                TurnSignalInput = vehicle.Signal;

            if (currentControlMode == VehicleControlMode.AUTONOMOUS)
            {
                if (Mathf.Abs(horizontal) > 0 || Mathf.Abs(vertical) > 0)
                {
                    Overriden = true;
                    NewControlMode = VehicleControlMode.MANUAL;
                }
            }


        }
    }
}