using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    [RequireComponent(typeof(VehicleOverrideInputManager))]
    public class VehicleInputBase : MonoBehaviour
    {
        public float AccelerationInput { get; protected set; } = 0;

        public float SteeringInput { get; protected set; } = 0;

        public Vehicle.Shift ShiftInput { get; protected set; } = Vehicle.Shift.PARKING;

        public Vehicle.TurnSignal TurnSignalInput { get; protected set; } = Vehicle.TurnSignal.NONE;

        public bool Overriden { get; protected set; } = false;

        public VehicleControlMode NewControlMode { get; protected set; } = VehicleControlMode.AUTONOMOUS;

        public virtual void OnUpdate(VehicleControlMode currentControlMode)
        {

        }
    }
}