using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Runtime.InteropServices;
using tier4_vehicle_msgs.msg;
using UnityEngine;
using UnityEngine.InputSystem;

namespace AWSIM
{
    /// <summary>
    /// Input Class for Logitech G29 Racing Wheel.
    /// During autonomous driving, FFB(force-feed back) is applied to reflect the steering angle on the steering wheel.
    /// When throttle, brake pedal or steering exceeds the set threshold value, an input override is determinded.
    /// </summary>
    /// ----- key binds -----
    /// Throttle        : Throttle pedal
    /// Brake           : Brake pedal
    /// Steering        : Steering wheel
    /// D gear          : Triangle button
    /// P gear          : Cross button
    /// R gear          : Square button
    /// N gear          : Circle button
    /// Left signal     : Left D-pad
    /// Right signal    : Right D-pad
    /// Hazard signal   : Up D-pad
    /// None signal     : Down D-pad
    public class VehicleG29Input : VehicleInputBase
    {
        class PIDController
        {
            public float Kp { get; set; }
            public float Ki { get; set; }
            public float Kd { get; set; }
            float previousError = 0f;
            float integral = 0f;

            public PIDController(float kp, float ki, float kd)
            {
                Kp = kp;
                Ki = ki;
                Kd = kd;
                previousError = 0f;
                integral = 0f;
            }

            public float Compute(float setpoint, float actualValue, float deltaTime)
            {
                float error = setpoint - actualValue;
                integral += error * deltaTime;
                float derivative = (error - previousError) / deltaTime;
                previousError = error;
                float direction = error < 0.0 ? -1.0f : 1.0f;
                var result = Kp * error + Ki * integral + Kd * derivative;
                result = Mathf.Clamp(Mathf.Abs(result), 0, 1) * direction;

                return result;
            }
        }

        public float MaxAcceleration = 1.5f;
        float MaxSteerAngle = 0;
        [SerializeField] Vehicle vehicle;

        public bool IsConnected { get; private set; } = false;


        [Header("G29 settings")]
        public string DevicePath = "/dev/input/event8";
        [SerializeField] float AllowableDiff = 0.04f;
        [SerializeField] float kp = 4f;
        [SerializeField] float ki = 0.2f;
        [SerializeField] float kd = 0.02f;
        PIDController pidController;

        [Header("Override settings")]
        [SerializeField] float accelerationThreshold = 0.05f;
        [SerializeField] float steeringThreshold = 0.3f;

        bool isInitialized = false;

        void OnEnable()
        {
            Initialize();
        }

        void Initialize()
        {
            if (isInitialized)
                return;

            MaxSteerAngle = vehicle.MaxSteerAngleInput;
            pidController = new PIDController(kp, ki, kd);
            IsConnected = G29Linux.InitDevice(DevicePath);

            isInitialized = false;
        }

        public override void OnUpdate(InputArg inputArg)
        {
            var currentControlMode = inputArg.VehicleControlMode;

            // Calculate ffb torque that can follow targetPos by PID.
            var targetPos = vehicle.SteerAngleNormalized;
            var currentPos = (float)G29Linux.GetPos();
            var pidResuleRate = pidController.Compute(targetPos, currentPos, Time.deltaTime);   // min:-1, none:0, max:1

            var minNormaizedTorque = 0.2f;  // min torque that g29 can output.

            // Apply the lowest torque that logitech g29 can output in FFB.
            var steerDiff = Mathf.Abs(targetPos - currentPos);
            var sign = Mathf.Sign(pidResuleRate);
            if (steerDiff < AllowableDiff)
            {
                minNormaizedTorque = 0f;
            }
            var clamped = Mathf.Clamp(Mathf.Abs(pidResuleRate), minNormaizedTorque, 1f);
            var finalNormalizedTorque = sign * clamped;

            // Branching by ControlMode.
            if (currentControlMode == VehicleControlMode.MANUAL)
            {
                // None ffb.
                G29Linux.UploadEffect(0, 0);
                SteeringInput = currentPos * MaxSteerAngle;
            }
            else if (currentControlMode == VehicleControlMode.AUTONOMOUS)
            {
                // Apply ffb.
                G29Linux.UploadEffect(finalNormalizedTorque, Time.deltaTime);

                // Override the vehicle's control mode when a steering input or acceleration input is given.
                if (Mathf.Abs(AccelerationInput) > accelerationThreshold || steerDiff > steeringThreshold)
                {
                    Overridden = true;
                    NewControlMode = VehicleControlMode.MANUAL;
                    ShiftInput = vehicle.AutomaticShift;
                    TurnSignalInput = vehicle.Signal;

                    G29Linux.UploadEffect(0, 0);
                }
            }
        }

        // Fuctions called from player input event.
        public void OnThrottle(InputAction.CallbackContext context)
        {
            var throttle = context.ReadValue<float>();
            AccelerationInput = MaxAcceleration * throttle;
        }

        public void OnBrake(InputAction.CallbackContext context)
        {
            var brake = context.ReadValue<float>();
            AccelerationInput = MaxAcceleration * -brake;
        }

        public void OnDriveGear(InputAction.CallbackContext context)
        {
            if (ShiftInput != Vehicle.Shift.DRIVE)
                ShiftInput = Vehicle.Shift.DRIVE;
        }

        public void OnReverseGear(InputAction.CallbackContext context)
        {
            if (ShiftInput != Vehicle.Shift.REVERSE)
                ShiftInput = Vehicle.Shift.REVERSE;
        }

        public void OnNeutralGear(InputAction.CallbackContext context)
        {
            if (ShiftInput != Vehicle.Shift.NEUTRAL)
                ShiftInput = Vehicle.Shift.NEUTRAL;
        }

        public void OnParkingGear(InputAction.CallbackContext context)
        {
            if (ShiftInput != Vehicle.Shift.PARKING)
                ShiftInput = Vehicle.Shift.PARKING;
        }

        public void OnTurnSignalNone(InputAction.CallbackContext context)
        {
            if (TurnSignalInput != Vehicle.TurnSignal.NONE)
                TurnSignalInput = Vehicle.TurnSignal.NONE;
        }

        public void OnTurnSignalLeft(InputAction.CallbackContext context)
        {
            if (TurnSignalInput != Vehicle.TurnSignal.LEFT)
                TurnSignalInput = Vehicle.TurnSignal.LEFT;
        }

        public void OnTurnSignalRight(InputAction.CallbackContext context)
        {
            if (TurnSignalInput != Vehicle.TurnSignal.RIGHT)
                TurnSignalInput = Vehicle.TurnSignal.RIGHT;
        }

        public void OnTurnSignalHazard(InputAction.CallbackContext context)
        {
            if (TurnSignalInput != Vehicle.TurnSignal.HAZARD)
                TurnSignalInput = Vehicle.TurnSignal.HAZARD;
        }
    }
}