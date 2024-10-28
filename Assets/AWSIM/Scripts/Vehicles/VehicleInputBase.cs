using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// Input base class to the vehicle.
    /// By creating arbitrary input classes to the vehicle (keyboard, steering controller, etc.) 
    /// that inherit from this class, you can reproduce actual vehicle input overrides.
    /// Actual vehicle input is performed by VehicleOverrideInputManager. 
    /// </summary>
    public class VehicleInputBase : MonoBehaviour
    {
        public struct InputArg
        {
            public VehicleControlMode VehicleControlMode { get; private set; }

            public InputArg(VehicleControlMode vehicleControlMode, float autonomousSteerInput)
            {
                VehicleControlMode = vehicleControlMode;
            }
        }

        /// <summary>
        /// Acceleration(m/s^2) to be applied to Vehicle acquired from Input.
        /// </summary>
        public float AccelerationInput { get; protected set; } = 0;

        /// <summary>
        /// Steering to be applied to Vehicle acquired from Input.
        /// </summary>
        public float SteeringInput { get; protected set; } = 0;

        /// <summary>
        /// Shift to be applied to Vehicle acquired from Input.
        /// </summary>
        public Vehicle.Shift ShiftInput { get; protected set; } = Vehicle.Shift.PARKING;

        /// <summary>
        /// TurnSignal to be applied to Vehicle acquired from Input.
        /// </summary>
        public Vehicle.TurnSignal TurnSignalInput { get; protected set; } = Vehicle.TurnSignal.NONE;

        /// <summary>
        /// Is there an input override to the vehicle?
        /// </summary>
        public bool Overridden { get; protected set; } = false;

        /// <summary>
        /// New control mode when input override occurs.
        /// Used by VehicleOverrideInputManager class when Overridden property is true.
        /// </summary>
        public VehicleControlMode NewControlMode { get; protected set; } = VehicleControlMode.AUTONOMOUS;

        /// <summary>
        /// Instead of unity callback Update(), this method is overridden and used to get inputs.
        /// This method is called by the VehicleOverrideInputManager class.
        /// </summary>
        /// <param name="currentControlMode"></param>
        public virtual void OnUpdate(InputArg currentControlMode)
        {
            // Override this method to get inputs!
        }
    }
}