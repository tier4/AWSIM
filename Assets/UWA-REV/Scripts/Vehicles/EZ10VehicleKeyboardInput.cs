using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// This is a sample class for controlling a ez10Vehicle with a keyboard.
    /// </summary>

    // ----- key binds -----
    // up arrow : Accelerate
    // down arrow : Deceleration
    // left/right arrow : Steering
    // D : Drive gear
    // P : Parking gear
    // R : Reverse gear
    // N : Neutral gear
    // 1 : Left turn signal
    // 2 : Right turn signal
    // 3 : Hazard
    // 4 : Turn signal off
    [RequireComponent(typeof(EZ10Vehicle))]
    public class EZ10VehicleKeyboardInput : MonoBehaviour
    {
        [SerializeField] EZ10Vehicle ez10Vehicle;

        [SerializeField] float maxAcceleration = 1.5f;
        [SerializeField] float maxSteerAngle = 35;

        void Reset()
        {
            if (ez10Vehicle == null)
                ez10Vehicle = GetComponent<EZ10Vehicle>();
        }

        void Update()
        {
            // get arrow inputs
            var horizontal = Input.GetAxis("Horizontal");
            var vertical = Input.GetAxis("Vertical");

            // set acceleration
            ez10Vehicle.AccelerationInput = maxAcceleration * vertical;

            // set steer
            ez10Vehicle.SteerAngleInput = maxSteerAngle * horizontal;

            // set gear
            if (Input.GetKey(KeyCode.D))
                ez10Vehicle.AutomaticShiftInput = EZ10Vehicle.Shift.DRIVE;
            else if (Input.GetKey(KeyCode.P))
                ez10Vehicle.AutomaticShiftInput = EZ10Vehicle.Shift.PARKING;
            else if (Input.GetKey(KeyCode.R))
                ez10Vehicle.AutomaticShiftInput = EZ10Vehicle.Shift.REVERSE;
            else if (Input.GetKey(KeyCode.N))
                ez10Vehicle.AutomaticShiftInput = EZ10Vehicle.Shift.NEUTRAL;

            // set turn signal
            if (Input.GetKey(KeyCode.Alpha1))
                ez10Vehicle.SignalInput = EZ10Vehicle.TurnSignal.LEFT;
            else if (Input.GetKey(KeyCode.Alpha2))
                ez10Vehicle.SignalInput = EZ10Vehicle.TurnSignal.RIGHT;
            else if (Input.GetKey(KeyCode.Alpha3))
                ez10Vehicle.SignalInput = EZ10Vehicle.TurnSignal.HAZARD;
            else if (Input.GetKey(KeyCode.Alpha4))
                ez10Vehicle.SignalInput = EZ10Vehicle.TurnSignal.NONE;
        }
    }
}