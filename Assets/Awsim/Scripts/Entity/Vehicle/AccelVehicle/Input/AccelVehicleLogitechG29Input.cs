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
        [System.Serializable]
        public class Settings
        {
            [SerializeField] string _devicePath;
            [SerializeField] float _selfAligningTorqueCoeff;

            public string DevicePath => _devicePath;
            public float SteeringTorqueCoeff => _selfAligningTorqueCoeff;

            // Default constructor for JsonUtility
            public Settings()
            {
            }

            // Constructor for manual instantiation
            public Settings(string devicePath, float selfAligningTorqueCoeff)
            {
                this._devicePath = devicePath;
                this._selfAligningTorqueCoeff = selfAligningTorqueCoeff;
            }
        }

        public string Name => "Logitech G29";
        public float AccelerationInput { get; private set; } = 0;
        public float SteerAngleInput { get; private set; } = 0;
        public Gear GearInput { get; private set; } = Gear.Drive;
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
        [SerializeField] float _selfAligningTorqueSpeedCoeff = 5.2f;
        [SerializeField] float _selfAligningTorqueSteerCoeff = 1.8f;
        [SerializeField, Range(0f, 0.1f)] float _stationarySteeringResistance = 0.1f;  // Steering resistance when stationary (0 - 0.1)

        [Header("Vehicle settings")]
        [SerializeField] Component _readonlyVehicleComponent = null;
        [SerializeField] AccelVehicleControlModeBasedInputter _controlModeBasedInputProvider = null;

        IReadOnlyAccelVehicle _readonlyVehicle = null;
        PidController _pidController;

        float _throttlePedalInput = 0;
        float _brakePedalInput = 0;
        bool _steeringOverride = false;

        // For steering control when stationary
        float _previousSteeringPos = 0;
        float _steeringVelocity = 0;
        const float _steeringVelocityThreshold = 0.01f;  // Velocity threshold to consider the steering wheel is moving


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

        public void Initialize(Settings settings)
        {
            _devicePath = settings.DevicePath;

            // Linear interpolation based on selfAligningTorqueCoeff
            // When coeff = 1: speedCoeff = 4, steerCoeff = 1
            // When coeff = 0: speedCoeff = 20, steerCoeff = 10
            var coeff = Mathf.Clamp01(settings.SteeringTorqueCoeff);
            if (coeff == 0)
            {
                _selfAligningTorqueSpeedCoeff = 0;
                _selfAligningTorqueSteerCoeff = 0;
            }
            else
            {
                _selfAligningTorqueSpeedCoeff = Mathf.Lerp(10f, 4f, coeff);
                _selfAligningTorqueSteerCoeff = Mathf.Lerp(5f, 1f, coeff);
            }


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
                SteerAngleInput = _readonlyVehicle.MaxSteerTireAngleInput * currentPos;

                // Pedal.
                AccelerationInput = _readonlyVehicle.MaxAccelerationInput * _throttlePedalInput;
                AccelerationInput += _readonlyVehicle.MaxDecelerationInput * _brakePedalInput * -1;

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
                    var speedFactor = Mathf.Clamp01(speedAbs / _selfAligningTorqueSpeedCoeff); // Maximum coefficient 1.0 at specified speed
                    var steeringFactor = Mathf.Clamp01(steeringAngleAbs / _selfAligningTorqueSteerCoeff); // Maximum coefficient 1.0 at specified angle

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
                    // When stationary: Apply resistance force only when steering wheel is moving
                    currentPos = (float)LogitechG29Linux.GetPos();

                    // Calculate steering velocity
                    if (Time.deltaTime > 0)
                    {
                        _steeringVelocity = (currentPos - _previousSteeringPos) / Time.deltaTime;
                    }
                    _previousSteeringPos = currentPos;

                    // Apply resistance force only when steering wheel is moving
                    if (Mathf.Abs(_steeringVelocity) > _steeringVelocityThreshold && _stationarySteeringResistance > 0)
                    {
                        // Apply resistance force in opposite direction of movement (damping effect)
                        var sign = -Mathf.Sign(_steeringVelocity);
                        var resistanceTorque = _stationarySteeringResistance;
                        var finalTorque = sign * resistanceTorque;

                        LogitechG29Linux.UploadEffect(finalTorque, Time.deltaTime);
                    }
                    else
                    {
                        // Do not apply force when steering wheel is stationary (maintain position)
                        LogitechG29Linux.UploadEffect(0, Time.deltaTime);
                    }
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