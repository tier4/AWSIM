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
using System;
using Awsim.Common;

namespace Awsim.Entity
{
    /// <summary>
    /// NPC Vehicle class.
    /// Controlled by Position and Rotation.
    /// </summary>
    public class PoseVehicle : MonoBehaviour
    {
        public enum TurnSignalState
        {
            Off,
            Left,
            Right,
            Hazard,
        }

        [Serializable]
        public class Wheel
        {
            /// <summary>
            /// Wheel contact with ground? Updated by UpdateGroundHit().
            /// </summary>
            public bool IsGrounded { get; private set; } = false;

            public WheelCollider WheelCollider => _wheelCollider;
            public Transform VisualTransform => _visualTransform;

            [SerializeField] WheelCollider _wheelCollider;
            [SerializeField] Transform _visualTransform;

            WheelHit _wheelHit;

            float _wheelPitchAngle = 0;
            float _lastSteerAngle = 0;

            public void UpdateWheelHit()
            {
                IsGrounded = _wheelCollider.GetGroundHit(out _wheelHit);
            }

            public void UpdateVisual(float speed, float steerAngle)
            {
                // Apply WheelCollider position to visual object.
                WheelCollider.GetWorldPose(out var pos, out _);
                VisualTransform.position = pos;

                // wheel forward rotation(pitch).
                var additionalPitchAngle = (speed * Time.deltaTime / WheelCollider.radius) * Mathf.Rad2Deg;
                _wheelPitchAngle += additionalPitchAngle;
                _wheelPitchAngle %= 360;

                // steer angle.
                var fixedSteerAngle = Mathf.MoveTowardsAngle(_lastSteerAngle, steerAngle, Time.deltaTime * _maxSteerSpeed);

                // Apply rotations to visual wheel object.
                VisualTransform.localEulerAngles = new Vector3(_wheelPitchAngle, fixedSteerAngle, 0);

                // Cache steer angle value for next update.
                _lastSteerAngle = fixedSteerAngle;
            }
        }

        [Serializable]
        public class Axle
        {
            public Wheel LeftWheel => _leftWheel;
            public Wheel RightWheel => _rightWheel;

            [SerializeField] Wheel _leftWheel;
            [SerializeField] Wheel _rightWheel;

            public void UpdateVisual(float speed, float steerAngle)
            {
                LeftWheel.UpdateVisual(speed, steerAngle);
                RightWheel.UpdateVisual(speed, steerAngle);
            }
        }

        [Serializable]
        public class AxleSettings
        {
            public bool IsGrounded { get; private set; } = false;

            [SerializeField] Axle _frontAxle;
            [SerializeField] Axle _rearAxle;

            public float GetWheelBase()
            {
                var frontPos = _frontAxle.LeftWheel.WheelCollider.transform.localPosition;
                var rearPos = _rearAxle.LeftWheel.WheelCollider.transform.localPosition;

                return Mathf.Abs(frontPos.z - rearPos.z);
            }

            public void UpdateIsGrounded()
            {
                _frontAxle.LeftWheel.UpdateWheelHit();
                _frontAxle.RightWheel.UpdateWheelHit();
                _rearAxle.LeftWheel.UpdateWheelHit();
                _rearAxle.RightWheel.UpdateWheelHit();

                IsGrounded = _frontAxle.LeftWheel.IsGrounded && _frontAxle.RightWheel.IsGrounded && _rearAxle.LeftWheel.IsGrounded && _rearAxle.RightWheel.IsGrounded;
            }

            public void UpdateVisual(float speed, float steerAngle)
            {
                _frontAxle.UpdateVisual(speed, steerAngle);
                _rearAxle.UpdateVisual(speed, 0);
            }
        }


        [Serializable]
        public class EmissionMaterial
        {
            [SerializeField] MeshRenderer _meshRenderer;
            [SerializeField] int _materialIndex;
            [SerializeField] float _lightingIntensity;
            [SerializeField] Color _lightingColor;
            [SerializeField, Range(0, 1)] float _lightingExposureWeight;

            Material _material = null;
            Color _defaultEmissiveColor;
            float _defaultExposureWeight;
            bool _isOn = false;

#if HDRP
            const string _emissiveColor = "_EmissiveColor";
            const string _emissiveExposureWeight = "_EmissiveExposureWeight";
#elif URP
            const string _emissiveColor = "_EmissionColor";
#endif

            public void Initialize()
            {
                if (_material != null)
                    return;

                _material = _meshRenderer.materials[_materialIndex];
                _material.EnableKeyword("_EMISSION");

#if HDRP
                _defaultEmissiveColor = _material.GetColor(_emissiveColor);
                _defaultExposureWeight = _material.GetFloat(_emissiveExposureWeight);
#elif URP
                _defaultEmissiveColor = _material.GetColor(_emissiveColor);
#endif
            }

            public void Set(bool isLightOn)
            {
                if (_isOn == isLightOn)
                    return;

                _isOn = isLightOn;

                var targetColor = isLightOn ? _lightingColor * _lightingIntensity : _defaultEmissiveColor;
#if HDRP
                _material.SetColor(_emissiveColor, targetColor);
                var exposureWeight = isLightOn ? _lightingExposureWeight : _defaultExposureWeight;
                _material.SetFloat(_emissiveExposureWeight, exposureWeight);
#elif URP
                _material.SetColor(_emissiveColor, targetColor);
#endif
            }

            public void DestroyMaterial()
            {
                if (_material != null)
                    UnityObject.DestroySafe(ref _material);
            }
        }

        // dynamics settings const values.
        const float _maxSteerAngle = 40f;                    // deg
        const float _maxSteerSpeed = 60f;                    // deg/s
        const float _maxVerticalSpeed = 40;                  // m/s
        const float _maxSlope = 45;                          // deg

        // light visual settings const values.
        const float _turnSignalBlinkSec = 0.5f;             // seconds
        const float _brakeLightAccelThreshold = -0.1f;      // m/s

        /// <summary>
        /// Current visualObject's activeself
        /// </summary>
        public bool VisualObjectRootActiveSelf => _visualObjectRoot.activeSelf;

        /// <summary>
        /// Vehicle bounding box.
        /// </summary>
        public Bounds Bounds => _bounds;

        [SerializeField]
        GameObject _visualObjectRoot;

        [Header("Physics Settings")]
        [SerializeField] Transform _centerOfMass;
        [SerializeField] Rigidbody _rigidbody;
        [SerializeField] Rigidbody trailer = null;
        [SerializeField] AxleSettings axleSettings;

        [Header("Bounding box Settngs")]
        [SerializeField] Bounds _bounds;

        [Header("Brake light parameters")]
        [SerializeField] EmissionMaterial _brakeLight;

        [Header("Turn signal parameters")]
        [SerializeField] EmissionMaterial _leftTurnSignalLight;
        [SerializeField] EmissionMaterial _rightTurnSignalLight;

        TurnSignalState _turnSignalState = TurnSignalState.Off;
        float _turnSignalTimer = 0;
        bool _currentTurnSignalOn = false;

        float _wheelbase;        // m
        float _acceleration;     // m/s^2
        Vector3 _velocity;       // m/s
        float _speed;            // m/s (forward only)
        float _yawAngularSpeed;  // deg/s (yaw only)

        Vector3 _lastVelocity;
        Vector3 _lastPosition;
        float _lastEulerAnguleY;
        float _lastSpeed;

        public Transform RigidBodyTransform => _rigidbody.transform;
        public Transform TrailerTransform => trailer?.transform;
        public Pose PoseInput { get; set; }
        public bool IsGrounded { get => axleSettings.IsGrounded; }

        public event Action<Collision> OnCollisionEnterAction;

        public void Initialize()
        {
            if (trailer == null)
                trailer = _rigidbody;
            _leftTurnSignalLight.Initialize();
            _rightTurnSignalLight.Initialize();
            _brakeLight.Initialize();

            _rigidbody.centerOfMass = transform.InverseTransformPoint(_centerOfMass.position);
            _lastPosition = _rigidbody.position;
            _wheelbase = axleSettings.GetWheelBase();

            PoseInput = new Pose(_rigidbody.position, _rigidbody.rotation);
        }

        /// Set <see cref="TurnSignalState"/> and turn on/off blinkers according to the value of <see cref="TurnSignalState"/>.
        /// </summary>
        /// <param name="state">New value of <see cref="TurnSignalState"/>.</param>
        /// <exception cref="InvalidEnumArgumentException">Exception when <paramref name="state"/> is invalid value.</exception>
        public void SetTurnSignalState(TurnSignalState turnSignalState)
        {
            if (this._turnSignalState != turnSignalState)
                this._turnSignalState = turnSignalState;
        }

        /// <summary>
        /// Visual objects on/off
        /// </summary>
        /// <param name="isActive">visual on/off</param>
        public void SetActiveVisualObjects(bool isActive)
        {
            if (_visualObjectRoot.activeSelf == isActive)
                return;

            _visualObjectRoot.SetActive(isActive);
        }

        public void OnUpdate()
        {
            // Update Wheel visuals.
            var steerAngle = CalcSteerAngle(_speed, _yawAngularSpeed, _wheelbase);
            axleSettings.UpdateVisual(_speed, steerAngle);

            // brake light.
            var isBrakeLightOn = IsBrakeLightOn();
            _brakeLight.Set(isBrakeLightOn);

            // turn signal.
            if (IsAnyTurnSignalInputs() == false)
            {
                if (_turnSignalTimer != 0)
                    _turnSignalTimer = 0;

                if (_currentTurnSignalOn != false)
                    _currentTurnSignalOn = false;

                _leftTurnSignalLight.Set(false);
                _rightTurnSignalLight.Set(false);
            }

            _turnSignalTimer -= Time.deltaTime;
            if (_turnSignalTimer < 0f)
            {
                _turnSignalTimer = _turnSignalBlinkSec;
                _currentTurnSignalOn = !_currentTurnSignalOn;
            }

            var isLeftTurnSignalOn = IsLeftTurnSignalOn();
            _leftTurnSignalLight.Set(isLeftTurnSignalOn);

            var isRightTurnSignalOn = IsRightTurniSignalOn();
            _rightTurnSignalLight.Set(isRightTurnSignalOn);

            // --- inner methods ---
            static float CalcSteerAngle(float speed, float yawAngularSpeed, float wheelBase)
            {
                yawAngularSpeed *= Mathf.Deg2Rad;

                if (Mathf.Abs(yawAngularSpeed) < 0.01f || Mathf.Abs(speed) < 0.01f)
                {
                    return 0f;
                }
                var gyrationRadius = speed / Mathf.Tan(yawAngularSpeed);
                var yaw = Mathf.Asin(Mathf.Clamp(wheelBase / gyrationRadius, -1f, 1f)) * Mathf.Rad2Deg;
                yaw = Mathf.Clamp(yaw, -_maxSteerAngle, _maxSteerAngle);

                return yaw;
            }

            bool IsBrakeLightOn()
            {
                var isOn = false;
                if (_speed < 0.03f)
                    isOn = true;
                //if (acceleration < brakeLightAccelThreshold)
                //    isOn = true;

                return isOn;
            }

            bool IsAnyTurnSignalInputs()
            {
                return _turnSignalState == TurnSignalState.Left
                    || _turnSignalState == TurnSignalState.Right
                    || _turnSignalState == TurnSignalState.Hazard;
            }

            bool IsLeftTurnSignalOn()
            {
                return (_turnSignalState == TurnSignalState.Left
                    || _turnSignalState == TurnSignalState.Hazard)
                    && _currentTurnSignalOn;
            }

            bool IsRightTurniSignalOn()
            {
                return (_turnSignalState == TurnSignalState.Right
                    || _turnSignalState == TurnSignalState.Hazard)
                    && _currentTurnSignalOn;
            }
        }

        public void OnFixedUpdate()
        {
            ApplyPoseInput(PoseInput);

            // Calculate physical states for visual update.
            // velocity & speed.
            _velocity = (_rigidbody.position - _lastPosition) / Time.deltaTime;
            _speed = Vector3.Dot(_velocity, transform.forward);

            // accleration.
            _acceleration = (_speed - _lastSpeed) / Time.deltaTime;

            // yaw angular speed.
            _yawAngularSpeed = (_rigidbody.rotation.eulerAngles.y - _lastEulerAnguleY) / Time.deltaTime;

            // TODO: set WheelCollider steer angle?

            axleSettings.UpdateIsGrounded();

            // Cache current frame values.
            _lastPosition = _rigidbody.position;
            _lastVelocity = _velocity;
            _lastEulerAnguleY = _rigidbody.rotation.eulerAngles.y;
            _lastSpeed = _speed;
        }

        void ApplyPoseInput(Pose pose)
        {
            _rigidbody.MovePosition(new Vector3(pose.position.x, _rigidbody.position.y, pose.position.z));
            var velocityY = Mathf.Min(_rigidbody.linearVelocity.y, _maxVerticalSpeed);
            _rigidbody.linearVelocity = new Vector3(0, velocityY, 0);

            var inputAngles = pose.rotation.eulerAngles;
            var rigidbodyAngles = _rigidbody.rotation.eulerAngles;
            var pitch = ClampDegree360(rigidbodyAngles.x, _maxSlope);
            var roll = ClampDegree360(rigidbodyAngles.z, _maxSlope);
            _rigidbody.MoveRotation(Quaternion.Euler(pitch, inputAngles.y, roll));
            var angularVelocity = _rigidbody.angularVelocity;
            _rigidbody.angularVelocity = new Vector3(angularVelocity.x, 0f, angularVelocity.z);

            static float ClampDegree360(float value, float maxAbsValue)
            {
                if (value < 360f - maxAbsValue && value > 180f)
                {
                    return 360f - maxAbsValue;
                }

                if (value > maxAbsValue && value <= 180f)
                {
                    return maxAbsValue;
                }

                return value;
            }
        }

        // Draw bounding box 
        void OnDrawGizmos()
        {
            // Cache Gizmos default values.
            var cacheColor = Gizmos.color;
            var cacheMatrix = Gizmos.matrix;

            // Apply color and matrix.
            Gizmos.color = Color.white;
            Gizmos.matrix = Matrix4x4.TRS(transform.position, transform.rotation, transform.lossyScale);

            // Draw wire cube.
            Gizmos.DrawWireCube(_bounds.center, _bounds.size);

            // Return to default value.
            Gizmos.color = cacheColor;
            Gizmos.matrix = cacheMatrix;
        }

        void OnCollisionEnter(Collision collision)
        {
            OnCollisionEnterAction?.Invoke(collision);
        }

        void Reset()
        {
            if (_rigidbody == null)
                _rigidbody = GetComponent<Rigidbody>();
        }

        void OnValidate()
        {
            _rigidbody.isKinematic = false;
            _rigidbody.useGravity = true;
            _rigidbody.interpolation = RigidbodyInterpolation.Interpolate;
        }

        void OnDestroy()
        {
            _leftTurnSignalLight.DestroyMaterial();
            _rightTurnSignalLight.DestroyMaterial();
            _brakeLight.DestroyMaterial();
        }
    }
}
