using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// Controls vehicle override inputs.
    /// </summary>
    [RequireComponent(typeof(Vehicle))]
    public class VehicleOverrideInputManager : MonoBehaviour
    {
        public VehicleInputBase AutonomousInput;

        public VehicleInputBase ManuallyInput;

        public VehicleControlMode ControlMode = VehicleControlMode.AUTONOMOUS;

        [SerializeField] Vehicle vehicle;

        void Reset()
        {
            if (vehicle == null)
                vehicle = GetComponent<Vehicle>();
        }

        void Update()
        {
            AutonomousInput.OnUpdate();
            ManuallyInput.OnUpdate();

            ApplyInput();
        }

        void ApplyInput()
        {
            if (ControlMode == VehicleControlMode.AUTONOMOUS)
            {
                vehicle.AccelerationInput = AutonomousInput.AccelerationInput;
                vehicle.SteerAngleInput = AutonomousInput.SteeringInput;
                vehicle.AutomaticShiftInput = AutonomousInput.ShiftInput;
                vehicle.SignalInput = AutonomousInput.TurnSignalInput;
            }
            else if (ControlMode == VehicleControlMode.MANUAL)
            {
                vehicle.AccelerationInput = ManuallyInput.AccelerationInput;
                vehicle.SteerAngleInput = ManuallyInput.SteeringInput;
                vehicle.AutomaticShiftInput = ManuallyInput.ShiftInput;
                vehicle.SignalInput = ManuallyInput.TurnSignalInput;
            }
        }
    }
}