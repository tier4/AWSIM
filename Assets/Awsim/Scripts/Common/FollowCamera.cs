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

namespace Awsim.Common
{
    /// <summary>
    /// Camera script that follows the target.
    /// </summary>
    public class FollowCamera : MonoBehaviour
    {
        public bool EnableRotateAround { get => _enableRotateAround; set => _enableRotateAround = value; }

        public Transform Target { get => _target; set => _target = value; }

        [Space(10)]
        [Tooltip("Transform of object to follow")]
        [SerializeField] Transform _target;

        [Header("Base Settings")]
        [Tooltip("Base distance between the camera and the target object")]
        [SerializeField] float _distance = 10.0f;

        [Tooltip("Lateral offset of the camera position")]
        [SerializeField] float _offset = 0.0f;

        [Tooltip("Base height of the camera above the target object")]
        [SerializeField] float _height = 5.0f;

        [Tooltip("Camera height multiplier")]
        [SerializeField] float _heightMultiplier = 0.5f;

        [Space(10)]
        [Header("Camera Movement Settings")]
        [Tooltip("Toggle key between rotate around mode and follow mode")]
        [SerializeField] bool _enableRotateAround = false;

        [Space(10)]
        [Tooltip("Mouse movement sensitivity for camera rotation around the target")]
        [Range(0.0f, 100.0f)]
        [SerializeField] float _rotateAroundSensitivity = 16.0f;

        [Tooltip("Mouse movement sensitivity for camera height adjustment")]
        [Range(0.0f, 20.0f)]
        [SerializeField] float _heightAdjustmentSensitivity = 1.0f;

        [Tooltip("Mouse scroll wheel sensitivity for camera zoom")]
        [Range(50.0f, 300.0f)]
        [SerializeField] float _zoomSensitivity = 200f;

        [Space(10)]
        [Tooltip("Invert horizontal mouse movement")]
        [SerializeField] bool _invertHorzAxis = false;

        [Tooltip("Invert vertical mouse movement")]
        [SerializeField] bool _invertVertAxis = false;

        [Tooltip("Invert mouse scroll wheel")]
        [SerializeField] bool _invertScrollWheel = false;

        [Space(10)]
        [Tooltip("Maximum value of camera height")]
        [SerializeField] float _maxHeight = 10f;

        [Tooltip("Minimum value of camera distance to target object")]
        [SerializeField] float _minDistance = 1f;

        [Tooltip("Maximum value of camera distance to target object")]
        [SerializeField] float _maxDistance = 20f;

        float _distanceAdjustmentSpeed = 0f;
        float _currentDistance = 10.0f;
        float _heightDamping = 2.0f;
        float _rotateAroundSpeed = 0.0f;
        float _currentCameraDirection = 0.0f;

        float _currentHeight = 0f;
        float _heightAdjustmentSpeed = 0f;
        float _deltaHeight = 0f;

        public void Initialize()
        {
            _distanceAdjustmentSpeed = 0.0f;
            _currentDistance = _distance;
            _rotateAroundSpeed = 0.0f;
            _currentCameraDirection = 0.0f;
            _enableRotateAround = false;

            _currentHeight = _height;
            _heightAdjustmentSpeed = 0f;
            _deltaHeight = 0f;
        }

        public void OnUpdate()
        {
            if (Target == null)
                return;

            if (_enableRotateAround)
            {
                if (Input.GetKey(KeyCode.LeftShift))
                {
                    // front view
                    if (Input.GetKey(KeyCode.UpArrow))
                    {
                        _rotateAroundSpeed = 0f;
                        _heightAdjustmentSpeed = 0f;
                        _currentHeight = transform.position.y - _target.position.y;
                        _distanceAdjustmentSpeed = 0f;
                        _currentCameraDirection = 180f;
                    }
                    // back view
                    else if (Input.GetKey(KeyCode.DownArrow))
                    {
                        _rotateAroundSpeed = 0f;
                        _heightAdjustmentSpeed = 0f;
                        _currentHeight = transform.position.y - _target.position.y;
                        _distanceAdjustmentSpeed = 0f;
                        _currentCameraDirection = 0f;
                    }
                    // left view
                    else if (Input.GetKey(KeyCode.LeftArrow))
                    {
                        _rotateAroundSpeed = 0f;
                        _heightAdjustmentSpeed = 0f;
                        _currentHeight = transform.position.y - _target.position.y;
                        _distanceAdjustmentSpeed = 0f;
                        _currentCameraDirection = 90f;
                    }
                    // right view
                    else if (Input.GetKey(KeyCode.RightArrow))
                    {
                        _rotateAroundSpeed = 0f;
                        _heightAdjustmentSpeed = 0f;
                        _currentHeight = transform.position.y - _target.position.y;
                        _distanceAdjustmentSpeed = 0f;
                        _currentCameraDirection = 270f;
                    }

                    // get horizontal mouse movement for camera rotation
                    float mouseHorzAxis = Input.GetAxis("Mouse X");
                    if (Mathf.Abs(mouseHorzAxis) > 0.01f)
                    {
                        _rotateAroundSpeed = _rotateAroundSensitivity * mouseHorzAxis * _currentDistance * (_invertHorzAxis ? 1f : -1f);
                    }
                    else
                    {
                        _rotateAroundSpeed = 0f;
                    }

                    // get vertical mouse movement for camera height adjustment
                    float mouseVertAxis = Input.GetAxis("Mouse Y");
                    if (Mathf.Abs(mouseVertAxis) > 0.01f)
                    {
                        _heightAdjustmentSpeed = _heightAdjustmentSensitivity * mouseVertAxis * _currentDistance * (_invertVertAxis ? -1f : 1f);
                    }
                    else
                    {
                        _heightAdjustmentSpeed = 0f;
                    }

                    // get mouse scroll whell for camera zoom
                    float mouseScroll = Input.GetAxis("Mouse ScrollWheel");
                    if (Mathf.Abs(mouseScroll) > 0.01f)
                    {
                        _distanceAdjustmentSpeed = _zoomSensitivity * mouseScroll * (_invertScrollWheel ? 1f : -1f);
                    }
                    else
                    {
                        _distanceAdjustmentSpeed = 0f;
                    }

                }
                else
                {
                    _rotateAroundSpeed = 0f;
                    _heightAdjustmentSpeed = 0f;
                    _distanceAdjustmentSpeed = 0f;
                }
            }

#if UNITY_EDITOR
            if (Time.deltaTime == 0.0f)
                return;
#endif
            if (_target == null)
                return;

            // calculate additional rotation, height and distance for camera
            if (_enableRotateAround)
            {
                // include additional rotation for camera rotating around target
                _currentCameraDirection += _rotateAroundSpeed * Time.deltaTime / Time.timeScale;
                if (_currentCameraDirection > 360)
                {
                    _currentCameraDirection -= 360f;
                }
                if (_currentCameraDirection < -360f)
                {
                    _currentCameraDirection += 360f;
                }

                // include additional height for camera above target
                _deltaHeight = _heightAdjustmentSpeed * Time.deltaTime / Time.timeScale;
                if (_currentHeight + _deltaHeight > _maxHeight)
                {
                    _deltaHeight = _maxHeight - _currentHeight;
                    _currentHeight = _maxHeight;
                }
                else if (_currentHeight + _deltaHeight < _height)
                {
                    _deltaHeight = _height - _currentHeight;
                    _currentHeight = _height;
                }
                else
                {
                    _currentHeight += _deltaHeight;
                }

                // include additional distance between camera and target
                _currentDistance += _distanceAdjustmentSpeed * Time.deltaTime / Time.timeScale;
                _currentDistance = Mathf.Clamp(_currentDistance, _minDistance, _maxDistance);
            }
            // set camera position to base values
            else
            {
                _currentCameraDirection = 0f;
                _currentDistance = _distance;
                _currentHeight = _height;
                _deltaHeight = 0f;
            }

            float newHeight = _target.position.y + _currentHeight;
            float currentCameraHeight = transform.position.y + _deltaHeight;
            currentCameraHeight = Mathf.Lerp(currentCameraHeight, newHeight, _heightDamping * Time.deltaTime);

            // calculate rotation for camera
            float currentRotationAngle = _target.eulerAngles.y;
            Quaternion currentCameraRotation = Quaternion.Euler(0, currentRotationAngle, 0);
            currentCameraRotation *= Quaternion.Euler(0f, _currentCameraDirection, 0f);

            // set camera position and orientation
            Vector3 pos = _target.position;
            pos -= currentCameraRotation * Vector3.forward * _currentDistance + currentCameraRotation * Vector3.right * _offset;
            pos.y = currentCameraHeight;

            transform.position = pos;
            transform.LookAt(_target.position + Vector3.up * _height * _heightMultiplier);
        }
    }
}