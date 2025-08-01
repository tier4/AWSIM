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

using Awsim.Common;
using System;
using UnityEngine;

namespace Awsim.Entity
{
    public class ImuSensor : MonoBehaviour
    {
        public interface IReadOnlyOutputData
        {
            /// <summary>
            /// Measured acceleration (m/s^2).
            /// </summary>
            public Vector3 LinearAcceleration { get; }

            /// <summary>
            /// Measured angular velocity (rad/s).
            /// </summary>
            public Vector3 AngularVelocity { get; }
        }

        /// <summary>
        /// This data is output from ImuSensor at the OutputHz cycle.
        /// </summary>
        /// TODO: Combining readonly access and performance.
        public class OutputData : IReadOnlyOutputData
        {
            /// <summary>
            /// Measured acceleration (m/s^2).
            /// </summary>
            public Vector3 LinearAcceleration { get; set; }

            /// <summary>
            /// Measured angular velocity (rad/s).
            /// </summary>
            public Vector3 AngularVelocity { get; set; }

            public OutputData()
            {
                LinearAcceleration = new Vector3();
                AngularVelocity = new Vector3();
            }
        }

        /// <summary>
        /// Action used in sensor callbacks.
        /// </summary>
        public Action<IReadOnlyOutputData> OnOutput { get; set; } = null;

        /// <summary>
        /// Data output hz.
        /// Sensor processing and callbacks are called in this hz.
        /// </summary>
        public int OutputHz { get => _outputHz; }

        /// <summary>
        /// The bool value determines whether to gravity is considered or not.
        ///</summary>
        public bool EnableGravity { get => _enableGravity; }

        [SerializeField] int _outputHz = 30;         // Autoware imu sensor basically output at 30hz.
        [SerializeField] bool _enableGravity = true;
        Vector3 _lastPosition;                       // Previous frame position used for acceleration calculation.
        Vector3 _lastVelocity;                       // Previous frame velocity used for acceleration calculation in global coordinate system.
        Vector3 _lastLocalVelocity;                  // Previous frame velocity used for acceleration calculation.
        QuaternionD _lastRotation;                   // Previous frame rotation used for angular velocity calculation.
        OutputData _outputData = new OutputData();
        Vector3 _g;                                  // Gravity considered in measuring of acceleration and angular velocity.

        public void Initialize()
        {
            _lastRotation = new QuaternionD(transform.rotation);
            _lastPosition = transform.position;

            if (EnableGravity == true)
            {
                _g = Physics.gravity;
            }
            else
            {
                _g = Vector3.zero;
            }

            // NOTE: Might be more appropriate timing to loop it in a separate thread.
            InvokeRepeating(nameof(Output), 0f, 1.0f / OutputHz);
        }

        public void Initialize(int outputHz, bool enableGravity)
        {
            _outputHz = outputHz;
            _enableGravity = enableGravity;

            Initialize();
        }

        public void OnFixedUpdate()
        {
            // Compute angular velocity.
            var currentRotation = new QuaternionD(transform.rotation);
            var deltaRotation = currentRotation * QuaternionD.Inverse(_lastRotation);
            deltaRotation.ToAngleAxis(out var angle, out var axis);
            var angularVelocity = (1.0f / Time.deltaTime) * (float)angle * axis;
            var localAngularVelocity = transform.InverseTransformDirection(angularVelocity);
            _lastRotation = currentRotation;

            // Compute acceleration.
            var Velocity = (transform.position - _lastPosition) / Time.deltaTime;
            var localVelocity = (transform.InverseTransformDirection(transform.position - _lastPosition)) / Time.deltaTime;
            var localAcceleration = transform.InverseTransformDirection((Velocity - _lastVelocity) / Time.deltaTime + _g);
            _lastPosition = transform.position;
            _lastVelocity = Velocity;
            _lastLocalVelocity = localVelocity;

            // TODO: Temporarily avoid NaN values. Needs investigation.
            if (float.IsNaN(localAngularVelocity.x) || float.IsNaN(localAngularVelocity.y) || float.IsNaN(localAngularVelocity.z))
                localAngularVelocity = Vector3.zero;

            _outputData.LinearAcceleration = localAcceleration;
            _outputData.AngularVelocity = localAngularVelocity;
        }

        void Output()
        {
            // Calls registered callbacks.
            OnOutput?.Invoke(_outputData);
        }
    }
}