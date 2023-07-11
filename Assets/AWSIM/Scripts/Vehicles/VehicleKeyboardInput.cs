using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Utils;

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
    [RequireComponent(typeof(Vehicle))]
    public class VehicleKeyboardInput : MonoBehaviour
    {
        [SerializeField] Vehicle vehicle;

        //[SerializeField] float maxAcceleration = 0.1f;
        [SerializeField] float maxSteerAngle = 35;

        void Reset()
        {
            if (vehicle == null)
                vehicle = GetComponent<Vehicle>();
        }

        void Update()
        {
            // get arrow inputs
            //var horizontal = Input.GetAxis("Horizontal");
            //var vertical = Input.GetAxis("Vertical");

            // set acceleration
            //vehicle.AccelerationInput = maxAcceleration * vertical;

            vehicle.AccelerationInput = 4.2f;

            if (Mathf.Sqrt(Mathf.Pow(vehicle.LocalVelocity.x, 2) + Mathf.Pow(vehicle.LocalVelocity.z, 2)) >= 10f)
                vehicle.AccelerationInput = 0.5f;
            

            // set steer
            //vehicle.SteerAngleInput = maxSteerAngle * horizontal;

            vehicle.SteerAngleInput = maxSteerAngle * 1.0f;

            vehicle.AutomaticShiftInput = Vehicle.Shift.DRIVE;

            // set gear
            if (Input.GetKey(KeyCode.D))
                vehicle.AutomaticShiftInput = Vehicle.Shift.DRIVE;
            else if (Input.GetKey(KeyCode.P))
                vehicle.AutomaticShiftInput = Vehicle.Shift.PARKING;
            else if (Input.GetKey(KeyCode.R))
                vehicle.AutomaticShiftInput = Vehicle.Shift.REVERSE;
            else if (Input.GetKey(KeyCode.N))
                vehicle.AutomaticShiftInput = Vehicle.Shift.NEUTRAL;

            // set turn signal
            if (Input.GetKey(KeyCode.Alpha1))
                vehicle.SignalInput = Vehicle.TurnSignal.LEFT;
            else if (Input.GetKey(KeyCode.Alpha2))
                vehicle.SignalInput = Vehicle.TurnSignal.RIGHT;
            else if (Input.GetKey(KeyCode.Alpha3))
                vehicle.SignalInput = Vehicle.TurnSignal.HAZARD;
            else if (Input.GetKey(KeyCode.Alpha4))
                vehicle.SignalInput = Vehicle.TurnSignal.NONE;

            //Debug.Log(vehicle.AccelerationInput);           
        }
    }
}