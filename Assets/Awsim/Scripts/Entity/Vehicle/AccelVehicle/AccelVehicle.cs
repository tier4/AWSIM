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

using System;
using UnityEngine;
using Awsim.Common;

namespace Awsim.Entity
{
    public class AccelVehicle : MonoBehaviour, IReadOnlyAccelVehicle
    {
        /// <summary>
        /// WheelCollider wrapper and update visual of Wheel. Use only WheelCollider collision and suspension.
        /// </summary>
        [Serializable]
        public class Wheel
        {
            /// <summary>
            /// Wheel contact with ground? Updated by UpdateGroundHit().
            /// </summary>
            public bool IsGrounded { get; private set; } = true;

            /// <summary>
            /// Data on the wheel hits. Updated by UpdateGroundHit().
            /// </summary>
            public WheelHit WheelHit
            {
                get { return _wheelHit; }
            }

            /// <summary>
            /// Tire angle (degree). Apply to front Axle only.
            /// </summary>
            /// NOTE: It might be nice to have Axle flag Steerable as well. And reflect angle on all Steerable.
            public float SteerTireAngle
            {
                get
                {
                    return _tierYawAngle;
                }

                set
                {
                    _tierYawAngle = value;

                    if (_tierYawAngle == 0)
                        SteerTireAngle = 0.00001f;

                    if (_wheelCollider.steerAngle != SteerTireAngle)
                        _wheelCollider.steerAngle = _tierYawAngle;
                }
            }

            /// <summary>
            /// Load on wheel (kg).
            /// </summary>
            public float SprungMass => _wheelCollider.sprungMass;

            [SerializeField] WheelCollider _wheelCollider;
            [SerializeField] Transform _wheelVisualTransform;
            WheelHit _wheelHit;
            Rigidbody _vehicleRigidbody;
            float _tierYawAngle = 0f;
            float _wheelPitchAngle = 0f;

            /// <summary>
            /// Initlaize wheel.
            /// </summary>
            public void Initialize()
            {
                _vehicleRigidbody = _wheelCollider.attachedRigidbody;

                // Initialize WheelCollider appropriately.
                // AccelerationBasedVehicle uses only WheelCollider's suspension and collision.
                // Turn off other functions.
                _wheelCollider.forwardFriction = new WheelFrictionCurve();
                _wheelCollider.sidewaysFriction = new WheelFrictionCurve();
                _wheelCollider.ConfigureVehicleSubsteps(1000.0f, 1, 1);
                _wheelCollider.motorTorque = 0.00001f;
            }

            /// <summary>
            /// Update wheel hit and grounded flag.
            /// </summary>
            public void UpdateWheelHit()
            {
                IsGrounded = _wheelCollider.GetGroundHit(out _wheelHit);
            }

            /// <summary>
            /// Update wheel visual object.
            /// </summary>
            public void UpdateWheelVisual(float forwardSlipMultiplier)
            {
                if (_wheelVisualTransform == null)
                    return;

                var vehicleVelocity = _vehicleRigidbody.linearVelocity;
                var localSpeed = _vehicleRigidbody.transform.InverseTransformDirection(vehicleVelocity);

                // Wheel position.
                _wheelCollider.GetWorldPose(out var pos, out _);
                _wheelVisualTransform.position = pos;

                // Wheel forward rotation (pitch).
                var addtionalPitchAngle = localSpeed.z / forwardSlipMultiplier * Time.deltaTime / _wheelCollider.radius * Mathf.Rad2Deg;
                _wheelPitchAngle += addtionalPitchAngle;
                _wheelPitchAngle %= 360f;

                _wheelVisualTransform.localEulerAngles = new Vector3(_wheelPitchAngle, SteerTireAngle, 0);
            }
        }

        /// <summary>
        /// Axle with wheels on both sides (left and right).
        /// </summary>
        [Serializable]
        public class Axle
        {
            [SerializeField] Wheel _leftWheel;
            [SerializeField] Wheel _rightWheel;

            /// <summary>
            /// Left wheel.
            /// </summary>
            public Wheel LeftWheel => _leftWheel;

            /// <summary>
            /// Right wheel.
            /// </summary>
            public Wheel RightWheel => _rightWheel;
        }

        [Serializable]
        public class Settings
        {
            [SerializeField] float _maxSteerTireAngleInput;
            [SerializeField] float _maxAccelerationInput;
            [SerializeField] float _maxDecelerationInput;

            public float MaxSteerTireAngle => _maxSteerTireAngleInput;
            public float MaxAccelerationInput => _maxAccelerationInput;
            public float MaxDecelerationInput => _maxDecelerationInput;

            // Default constructor for JsonUtility
            public Settings()
            {

            }

            public Settings(float maxSteerTireAngleInput, float maxAccelerationInput, float maxDecelerationInput)
            {
                this._maxSteerTireAngleInput = maxSteerTireAngleInput;
                this._maxAccelerationInput = maxAccelerationInput;
                this._maxDecelerationInput = maxDecelerationInput;
            }
        }


        // ----- Vehicle input properties -----

        /// <summary>
        /// Acceleration input (m/s^2).
        /// </summary>
        public float AccelerationInput { get; set; } = 0f;

        /// <summary>
        /// Steering tire angle input (degree).
        /// </summary>
        public float SteerTireAngleInput
        {
            get
            {
                return _steerTireAngle;
            }

            set
            {
                _steerTireAngle = Mathf.Clamp(value, _maxSteerTireAngleInput * -1.0f, _maxSteerTireAngleInput);
            }
        }

        /// <summary>
        /// Gear input (AT).
        /// </summary>
        public Gear GearInput { get; set; } = Gear.Parking;

        /// <summary>
        /// Turn indicators input.
        /// </summary>
        public TurnIndicators TurnIndicatorsInput { get; set; } = TurnIndicators.None;

        /// <summary>
        /// Hazard lights input.
        /// </summary>
        public HazardLights HazardLightsInput { get; set; } = HazardLights.Disable;

        /// <summary>
        /// Change the slip rate of the wheel in the forward direction of this car. 1 no slip, 0 is full slip.
        /// </summary>
        public float ForwardSlipMultiplier
        {
            get
            {
                return _forwardSlipMultiplier;
            }

            set
            {
                _forwardSlipMultiplier = Mathf.Max(0.0001f, value);
            }
        }

        /// <summary>
        /// Change the slip rate of the wheel in the sideway direction of this car. 1 no slip, 0 is full slip.
        /// </summary>
        public float SidewaySlipMultiplier
        {
            get
            {
                return _sidewaySlipMultiplier;
            }

            set
            {
                _sidewaySlipMultiplier = Mathf.Max(0.0001f, value);
            }
        }

        // ----- Vehicle state properties -----

        /// <summary>
        /// Gear (AT).
        /// </summary>
        public Gear Gear => GearInput;

        /// <summary>
        /// Turn indicators.
        /// </summary>
        public TurnIndicators TurnIndicators => TurnIndicatorsInput;

        /// <summary>
        /// Hazard lights.
        /// </summary>
        public HazardLights HazardLights => HazardLightsInput;

        /// <summary>
        /// Steering tire angle (degree).
        /// </summary>
        public float SteerTireAngle => _firstOrderLaggedSteerTireAngle.Value;

        /// <summary>
        /// Local acceleration (m/s^2).
        /// </summary>
        public Vector3 LocalAcceleration { get; private set; } = Vector3.zero;

        /// <summary>
        /// Vehicle angular velocity in the local coordinate system of the vehicle (rad/s).
        /// </summary>
        public Vector3 LocalAngularVelocity => _transform.InverseTransformDirection(AngularVelocity);

        /// <summary>
        /// Speed (m/s).
        /// </summary>
        public float Speed { get; private set; } = 0f;

        /// <summary>
        /// Velocity (m/s).
        /// </summary>
        public Vector3 Velocity => _rigidbody.linearVelocity;

        /// <summary>
        /// Local velcoity (m/s).
        /// </summary>
        public Vector3 LocalVelocity => _transform.InverseTransformDirection(Velocity);

        /// <summary>
        /// Angular velocity (rad/s).
        /// </summary>
        public Vector3 AngularVelocity { get; private set; } = Vector3.zero;

        public Vector3 LocalAngularAcceleration => _transform.InverseTransformDirection(AngularAcceleration);

        /// <summary>
        /// Angular acceleration (ras/s^2). 
        /// </summary>
        public Vector3 AngularAcceleration { get; private set; } = Vector3.zero;

        public Wheel[] Wheels => _wheels;

        /// <summary>
        /// Maximum steer angle that can be input (used for device input, etc.)
        /// </summary>
        public float MaxSteerTireAngleInput => _maxSteerTireAngleInput;

        /// <summary>
        /// Maximum input acceleration value (used for device inputs, etc.)
        /// </summary>
        public float MaxAccelerationInput => _maxAccelerationInput;

        /// <summary>
        /// Maximum deceleration value that can be input (used for device inputs, etc.)
        /// </summary>
        public float MaxDecelerationInput => _maxDecelerationInput;

        public float SteerTireAngleNormalized => SteerTireAngle / _maxSteerTireAngleInput;

        [SerializeField] Rigidbody _rigidbody;
        [SerializeField] float _skiddingCancelRate = 0.1f;
        [SerializeField] Axle _frontAxle;
        [SerializeField] Axle _rearAxle;
        [SerializeField] float sleepVelocityThreshold = 0.02f;
        [SerializeField] float _accelerationTimeConstant = 0f;
        [SerializeField] float _steerTireAngleTimeConstant = 0f;
        FirstOrderLaggedFloat _firstOrderLaggedAcceleration = null;
        FirstOrderLaggedFloat _firstOrderLaggedSteerTireAngle = null;
        [SerializeField] float _maxSteerTireAngleInput = 35f;
        [SerializeField] float _maxAccelerationInput = 1.5f;
        [SerializeField] float _maxDecelerationInput = 3.5f;
        float _steerTireAngle = 0f;

        [Tooltip("Deceleration curve when throttle is off (like engine braking). Time axis represents velocity (m/s), Value axis represents deceleration (m/s^2).")]
        [SerializeField] AnimationCurve _decelerationCurve;

        // Slip multiplier
        [SerializeField] float _forwardSlipMultiplier = 1f;
        [SerializeField] float _sidewaySlipMultiplier = 1f;


        // Cache components.
        Transform _transform = null;
        Wheel[] _wheels = null;

        // Cache previous frame values.
        Vector3 _lastVelocity = Vector3.zero;
        Quaternion _lastRotation = new Quaternion();
        Vector3 _lastAngularVelocity = Vector3.zero;

        /// <summary>
        /// Initialize vehicle.
        /// </summary>
        public void Initialize()
        {
            _transform = transform;
            _wheels = new Wheel[] { _frontAxle.LeftWheel, _frontAxle.RightWheel, _rearAxle.LeftWheel, _rearAxle.RightWheel };
            foreach (var wheel in _wheels)
                wheel.Initialize();

            _firstOrderLaggedAcceleration = new FirstOrderLaggedFloat(_accelerationTimeConstant, AccelerationInput);
            _firstOrderLaggedSteerTireAngle = new FirstOrderLaggedFloat(_steerTireAngleTimeConstant, SteerTireAngleInput);
        }

        public void Initialize(Settings settings)
        {
            _maxSteerTireAngleInput = settings.MaxSteerTireAngle;
            _maxAccelerationInput = settings.MaxAccelerationInput;
            _maxDecelerationInput = settings.MaxDecelerationInput;

            Initialize();
        }

        /// <summary>
        /// Update wheel visuals.
        /// Assumption called by Update() of MonoBehaviour.
        /// </summary>
        public void OnUpdate()
        {
            foreach (var wheel in _wheels)
                wheel.UpdateWheelVisual(ForwardSlipMultiplier);
        }

        /// <summary>
        /// Calculate vehicle states, apply first order lag, determine sleep physics components, and apply tire forces.
        /// Assumption called by Update() of MonoBehaviour.
        /// </summary>
        public void OnFixedUpdate()
        {
            // --- Calculate vehicle states ---
            // Speed.
            Speed = Vector3.Dot(Velocity, _transform.forward);

            // Local acceleration.
            Vector3 acceleration = (Velocity - _lastVelocity) / Time.deltaTime;
            LocalAcceleration = _transform.InverseTransformDirection(acceleration);
            _lastVelocity = Velocity;

            // Angular velocity and angular acceleration.
            _lastAngularVelocity = AngularVelocity;
            AngularVelocity = ((_transform.rotation.eulerAngles - _lastRotation.eulerAngles) / Time.deltaTime) * Mathf.Deg2Rad;
            AngularAcceleration = ((AngularVelocity - _lastAngularVelocity) / Time.deltaTime);

            // --- Apply first order lag ---
            _firstOrderLaggedAcceleration.DesiredValue = AccelerationInput;
            _firstOrderLaggedSteerTireAngle.DesiredValue = SteerTireAngleInput;
            _frontAxle.LeftWheel.SteerTireAngle = _firstOrderLaggedSteerTireAngle.Value;
            _frontAxle.RightWheel.SteerTireAngle = _firstOrderLaggedSteerTireAngle.Value;

            // --- Determine sleep physics ---
            foreach (var wheel in Wheels)
            {
                wheel.UpdateWheelHit();
            }

            var canSleep = CanSleep();

            if (canSleep)
            {
                Sleep();
                return;
            }
            else
            {
                WakeUp();
            }

            // --- Apply tire forces ---
            var a = _firstOrderLaggedAcceleration.Value;


            if (Gear == Gear.Drive)
            {
                ApplyWheelForce(a);
            }
            else if (Gear == Gear.Reverse)
            {
                ApplyWheelForce(-a);
            }
            else if (Gear == Gear.Neutral)
            {
                if (a >= 0)
                    ApplyWheelForce(0);
                else if (Speed > 0)
                    ApplyWheelForce(a);
                else if (Speed < 0)
                    ApplyWheelForce(-a);
            }

            bool CanSleep()
            {
                var isAllWheelGrounded = true;
                foreach (var e in Wheels)
                {
                    if (e.IsGrounded == false)
                    {
                        isAllWheelGrounded = false;
                        break;
                    }
                }
                var isParkingShift = Gear == Gear.Parking;
                var isNeutralShift = Gear == Gear.Neutral;
                var isDriveShift = Gear == Gear.Drive;
                var isReverseShift = Gear == Gear.Reverse;
                var isUnderSleepVelocity = Mathf.Abs(LocalVelocity.z) < sleepVelocityThreshold;
                var isZeroOrNegativeAcceleration = AccelerationInput <= 0;
                var isMinusSpeed = Speed < 0;
                var isPlusSpeed = Speed > 0;

                // Can sleep?
                var canSleepParkingShift = isAllWheelGrounded && isParkingShift;
                var canSleepDriveShift = isAllWheelGrounded && isDriveShift && isZeroOrNegativeAcceleration && (isMinusSpeed || isUnderSleepVelocity);
                var canSleepReverseShift = isAllWheelGrounded && isReverseShift && isZeroOrNegativeAcceleration && (isPlusSpeed || isUnderSleepVelocity);
                var canSleepNeutralShift = isAllWheelGrounded && isNeutralShift && isUnderSleepVelocity;
                var canSleep = canSleepParkingShift || canSleepDriveShift || canSleepNeutralShift || canSleepReverseShift;

                return canSleep;
            }

            void Sleep()
            {
                _rigidbody.linearVelocity = Vector3.zero;
                _rigidbody.angularVelocity = Vector3.zero;
                _rigidbody.Sleep();
                _rigidbody.constraints = RigidbodyConstraints.FreezeAll;
            }

            void WakeUp()
            {
                _rigidbody.constraints = RigidbodyConstraints.None;
                _rigidbody.WakeUp();
            }

            void ApplyWheelForce(float acceleration)
            {
                var eachWheelAcceleration = acceleration / Wheels.Length;

                var eachWheelDeceleration = -1 * Mathf.Sign(Speed) * _decelerationCurve.Evaluate(Mathf.Abs(Speed)) / Wheels.Length;
                if (Mathf.Abs(AccelerationInput) > 0)
                {
                    eachWheelDeceleration = 0;
                }

                foreach (var wheel in Wheels)
                {
                    if (wheel.IsGrounded == false)
                        return;

                    var pointVelocity = _rigidbody.GetPointVelocity(wheel.WheelHit.point);
                    var wheelVelocity = pointVelocity - Vector3.Project(pointVelocity, wheel.WheelHit.normal);
                    var localWheelVelocity = Vector3.zero;
                    localWheelVelocity.y = Vector3.Dot(wheel.WheelHit.forwardDir, wheelVelocity);
                    localWheelVelocity.x = Vector3.Dot(wheel.WheelHit.sidewaysDir, wheelVelocity);

                    Vector2 cancelForce = -1 * _skiddingCancelRate * wheel.SprungMass * localWheelVelocity / Time.fixedDeltaTime;
                    Vector3 skiddingCancelForce = wheel.WheelHit.sidewaysDir * cancelForce.x;

                    // Apply cancel force.
                    // Apply force to stand still against the lateral direction.
                    // TODO: Consideration of more accurate force calculation and processing methods.
                    var lateralCancelForce = skiddingCancelForce * SidewaySlipMultiplier;
                    _rigidbody.AddForceAtPosition(lateralCancelForce, wheel.WheelHit.point, ForceMode.Force);

                    // Apply drive force.
                    // Apply a force that will result in the commanded acceleration.
                    var driveForce = eachWheelAcceleration * wheel.WheelHit.forwardDir * ForwardSlipMultiplier;
                    _rigidbody.AddForceAtPosition(driveForce, wheel.WheelHit.point, ForceMode.Acceleration);

                    // Apply resistance force.
                    var resistForce = eachWheelDeceleration * wheel.WheelHit.forwardDir * ForwardSlipMultiplier;
                    _rigidbody.AddForceAtPosition(resistForce, wheel.WheelHit.point, ForceMode.Acceleration);
                }
            }
        }
    }
}