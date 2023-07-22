using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// This is a sample class for controlling a vehicle with a keyboard.
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
        [SerializeField] EZ10Vehicle vehicle;

        [SerializeField] float maxAcceleration = 0.75f;
        [SerializeField] float maxSteerAngle = 15;

        void Reset()
        {
            if (vehicle == null)
                vehicle = GetComponent<EZ10Vehicle>();
        }

        void Update()
        {
            // get arrow inputs
            var horizontal = Input.GetAxis("Horizontal");
            var vertical = Input.GetAxis("Vertical");

            // set acceleration
            vehicle.AccelerationInput = maxAcceleration * vertical;

            // set steer
            vehicle.SteerAngleInput = maxSteerAngle * horizontal;

            // set gear
            if (Input.GetKey(KeyCode.D))
                vehicle.AutomaticShiftInput = EZ10Vehicle.Shift.DRIVE;
            else if (Input.GetKey(KeyCode.P))
                vehicle.AutomaticShiftInput = EZ10Vehicle.Shift.PARKING;
            else if (Input.GetKey(KeyCode.R))
                vehicle.AutomaticShiftInput = EZ10Vehicle.Shift.REVERSE;
            else if (Input.GetKey(KeyCode.N))
                vehicle.AutomaticShiftInput = EZ10Vehicle.Shift.NEUTRAL;

            // set turn signal
            if (Input.GetKey(KeyCode.Alpha1))
                vehicle.SignalInput = EZ10Vehicle.TurnSignal.LEFT;
            else if (Input.GetKey(KeyCode.Alpha2))
                vehicle.SignalInput = EZ10Vehicle.TurnSignal.RIGHT;
            else if (Input.GetKey(KeyCode.Alpha3))
                vehicle.SignalInput = EZ10Vehicle.TurnSignal.HAZARD;
            else if (Input.GetKey(KeyCode.Alpha4))
                vehicle.SignalInput = EZ10Vehicle.TurnSignal.NONE;
        }
    }
}