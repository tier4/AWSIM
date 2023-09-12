using System;
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
    [RequireComponent(typeof(Vehicle))]
    public class VehicleKeyboardInput : MonoBehaviour
    {
        [SerializeField] Vehicle vehicle;

        [SerializeField] float maxAcceleration = 1.5f;
        [SerializeField] float maxSteerAngle = 35;

        struct MyTransform
        {
            public Vector3 position;
            public Quaternion rotation;
        }
        
        private MyTransform originalTransform;
        private MyTransform savedTransform;

        private void Start()
        {
            if (vehicle == null)
                vehicle = GetComponent<Vehicle>();
            originalTransform = new MyTransform();
            originalTransform.position = vehicle.transform.position;
            originalTransform.rotation = vehicle.transform.rotation;
            savedTransform = new MyTransform();
            savedTransform.position = vehicle.transform.position;
            savedTransform.rotation = vehicle.transform.rotation;
        }

        void Reset()
        {
            if (vehicle == null)
                vehicle = GetComponent<Vehicle>();
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

            // Save the current transform when F7 is pressed
            if (Input.GetKeyDown(KeyCode.F7))
            {
                savedTransform.position = vehicle.transform.position;
                savedTransform.rotation = vehicle.transform.rotation;
            }

            // Set the vehicle's transform to the saved transform when F5 is pressed
            if (Input.GetKeyDown(KeyCode.F5))
            {
                vehicle.transform.position = originalTransform.position;
                vehicle.transform.rotation = originalTransform.rotation;
            }

            if (Input.GetKeyDown(KeyCode.F6))
            {
                vehicle.transform.position = savedTransform.position;
                vehicle.transform.rotation = savedTransform.rotation;
            }
        }
    }
}