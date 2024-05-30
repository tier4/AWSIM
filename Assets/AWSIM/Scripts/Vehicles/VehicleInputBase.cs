using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    [RequireComponent(typeof(VehicleOverrideInputManager))]
    public class VehicleInputBase : MonoBehaviour
    {
        public float AccelerationInput { get; protected set; }

        public float SteeringInput { get; protected set; }
        
        public Vehicle.Shift ShiftInput { get; protected set; }

        public Vehicle.TurnSignal TurnSignalInput { get; protected set; }

        public bool Overriden { get; protected set; }

        public VehicleControlMode NewControlMode { get; protected set; }

        public virtual void OnUpdate()
        {
            
        }
    }
}