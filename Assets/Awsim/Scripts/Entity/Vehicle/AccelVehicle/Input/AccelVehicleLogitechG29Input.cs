// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using UnityEngine;
using UnityEngine.InputSystem;
using Awsim.Common;
using Unity.Mathematics;

namespace Awsim.Entity
{
    public class AccelVehicleLogitechG29Input : MonoBehaviour, IAccelVehicleInput
    {
        class PidController
        {
            public float Kp { get; set; }
            public float Ki { get; set; }
            public float Kd { get; set; }
            float previousError = 0f;
            float integral = 0f;

            public PidController(float kp, float ki, float kd)
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

        public string Name => "Logitech G29";
        public float AccelerationInput { get; private set; } = 0;
        public float SteerAngleInput { get; private set; } = 0;
        public Gear GearInput { get; private set; } = Gear.Parking;
        public TurnIndicators TurnIndicatorsInput { get; private set; } = TurnIndicators.None;
        public HazardLights HazardLightsInput { get; private set; } = HazardLights.Disable;
        public bool SwitchAutonomous { get; private set; } = false;
        public bool Connected { get; private set; }

        [Header("Logitech G29 settings")]
        [SerializeField] string _devicePath = "/dev/input/event6";
        [SerializeField] float _kp = 5f;
        [SerializeField] float _ki = 0.2f;
        [SerializeField] float _kd = 0.05f;
        [SerializeField] float _minNormalizedSteeringTorque = 0.17f;
        [SerializeField] float _speedCoeff = 10;

        [Header("Vehicle settings")]
        [SerializeField] Component _readonlyVehicleComponent = null;
        [SerializeField] AccelVehicleControlModeBasedInputter _controlModeBasedInputProvider = null;

        IReadOnlyAccelVehicle _readonlyVehicle = null;
        PidController _pidController;

        float _throttlePedalInput = 0;
        float _brakePedalInput = 0;
        bool _steeringOverride = false;


        public void Initialize()
        {
            // Linux only.
            if (!IsLinux())
                return;

            _readonlyVehicle = _readonlyVehicleComponent as IReadOnlyAccelVehicle;
            _pidController = new PidController(_kp, _ki, _kd);

            var connected = LogitechG29Linux.InitDevice(_devicePath);
            Connected = connected;
        }

        public void Initialize(string devicePath)
        {
            _devicePath = devicePath;

            Initialize();
        }

        public bool UpdateInputs()
        {
            if (!Connected)
                return false;

            var isOverridden = false;
            SwitchAutonomous = false;
            var currentControlMode = _controlModeBasedInputProvider.ControlMode;

            // ControlMode is manual.
            if (currentControlMode == ControlMode.Manual)
            {
                ManuallyInput();
            }
            // ControlMode is autonomous.
            else if (currentControlMode == ControlMode.Autonomous)
            {
                // TODO: It would be better if pedal input could also be used to determine override.
                var isSteerControlled = _steeringOverride;

                isOverridden = isSteerControlled;

                if (isOverridden)
                {
                    ManuallyInput();
                    if (_steeringOverride)
                        _steeringOverride = false;
                }
                else
                {
                    // Steering is controlled by FFB.
                    var targetPos = _readonlyVehicle.SteerTireAngleNormalized;
                    var currentPos = (float)LogitechG29Linux.GetPos();
                    var pidResultRate = _pidController.Compute(targetPos, currentPos, Time.deltaTime);
                    var sign = Mathf.Sign(pidResultRate);
                    var clamped = Mathf.Clamp(Mathf.Abs(pidResultRate), _minNormalizedSteeringTorque, 1f);
                    var finalNormalaizedTorque = sign * clamped;

                    LogitechG29Linux.UploadEffect(finalNormalaizedTorque, Time.deltaTime);
                }
            }

            void ManuallyInput()
            {
                if (_isOnSwitchAutonomous)
                {
                    SwitchAutonomous = true;
                    _isOnSwitchAutonomous = false;
                }

                // Steering.
                var currentPos = (float)LogitechG29Linux.GetPos();
                SteerAngleInput = _readonlyVehicle.MaxSteerTireAngle * currentPos;

                // Pedal.
                AccelerationInput = _readonlyVehicle.MaxAcceleration * _throttlePedalInput;
                AccelerationInput += _readonlyVehicle.MaxAcceleration * _brakePedalInput * -1;

                // Steering is controlled by FFB.

                if (Mathf.Abs(_readonlyVehicle.Speed) > 0)
                {
                    // Return steering to center based on vehicle speed
                    var targetPos = 0f; // Target center position
                    currentPos = (float)LogitechG29Linux.GetPos();

                    // Get absolute value of steering angle
                    var steeringAngleAbs = Mathf.Abs(currentPos);

                    // Get absolute value of speed (assuming km/h units)
                    var speedAbs = Mathf.Abs(_readonlyVehicle.Speed);

                    // Calculate return factor based on speed and steering angle
                    // Higher speed and larger steering angle result in faster return to center
                    var speedFactor = Mathf.Clamp01(speedAbs / _speedCoeff); // Maximum coefficient 1.0 at specified speed
                    var steeringFactor = Mathf.Clamp01(steeringAngleAbs / 1f); // Maximum coefficient 1.0 at specified angle

                    // Combine return factors (considering both factors)
                    var returnFactor = Mathf.Lerp(0.1f, 1.0f, speedFactor * steeringFactor);

                    // PID control for center return
                    var pidResultRate = _pidController.Compute(targetPos, currentPos, Time.deltaTime);
                    var sign = Mathf.Sign(pidResultRate);
                    var clamped = Mathf.Clamp(Mathf.Abs(pidResultRate), _minNormalizedSteeringTorque, 1f);

                    // Apply return factor to adjust torque
                    var adjustedTorque = clamped * returnFactor;
                    var finalNormalaizedTorque = sign * adjustedTorque;

                    LogitechG29Linux.UploadEffect(finalNormalaizedTorque, Time.deltaTime);
                }
                else
                {
                    // Keep steering position when speed is 0
                    LogitechG29Linux.UploadEffect(0, Time.deltaTime);
                }
            }

            return isOverridden;
        }


        bool _isOnSwitchAutonomous = false;     // TODO: better name

        public void OnSwitchAutonomous(InputAction.CallbackContext context)
        {
            _isOnSwitchAutonomous = true;
        }

        // Fuctions called from player input event.
        public void OnThrottle(InputAction.CallbackContext context)
        {
            _throttlePedalInput = context.ReadValue<float>();
        }

        public void OnBrake(InputAction.CallbackContext context)
        {
            _brakePedalInput = context.ReadValue<float>();
        }

        public void OnSteeringOverrideInput(InputAction.CallbackContext context)
        {
            _steeringOverride = true;
        }

        public void OnDriveGear(InputAction.CallbackContext context)
        {
            if (GearInput != Gear.Drive)
                GearInput = Gear.Drive;
        }

        public void OnReverseGear(InputAction.CallbackContext context)
        {
            if (GearInput != Gear.Reverse)
                GearInput = Gear.Reverse;
        }

        public void OnNeutralGear(InputAction.CallbackContext context)
        {
            if (GearInput != Gear.Neutral)
                GearInput = Gear.Neutral;
        }

        public void OnParkingGear(InputAction.CallbackContext context)
        {
            if (GearInput != Gear.Parking)
                GearInput = Gear.Parking;
        }

        public void OnTurnSignalNone(InputAction.CallbackContext context)
        {
            if (TurnIndicatorsInput != TurnIndicators.None)
                TurnIndicatorsInput = TurnIndicators.None;
        }

        public void OnTurnSignalLeft(InputAction.CallbackContext context)
        {
            if (TurnIndicatorsInput != TurnIndicators.Left)
                TurnIndicatorsInput = TurnIndicators.Left;
        }

        public void OnTurnSignalRight(InputAction.CallbackContext context)
        {
            if (TurnIndicatorsInput != TurnIndicators.Right)
                TurnIndicatorsInput = TurnIndicators.Right;
        }

        public void OnTurnSignalHazard(InputAction.CallbackContext context)
        {
            if (HazardLightsInput == HazardLights.Enable)
                HazardLightsInput = HazardLights.Disable;
            else
                HazardLightsInput = HazardLights.Enable;
        }

        bool IsLinux()
        {
            if (Application.platform == RuntimePlatform.LinuxEditor ||
                Application.platform == RuntimePlatform.LinuxPlayer)
                return true;
            else
                return false;
        }
    }
}