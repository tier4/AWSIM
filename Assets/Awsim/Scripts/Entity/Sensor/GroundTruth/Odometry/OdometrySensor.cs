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
using Awsim.Common;
using UnityEngine;

namespace Awsim.Entity
{
    public class OdometrySensor : MonoBehaviour
    {
        public interface IReadOnlyOutputData
        {
            public Vector3 Position { get; }
            public Quaternion Rotation { get; }
            public Vector3 LinearVelocity { get; }
            public Vector3 AngularVelocity { get; }
        }

        public class OutputData : IReadOnlyOutputData
        {
            public Vector3 Position { get; set; }
            public Quaternion Rotation { get; set; }
            public Vector3 LinearVelocity { get; set; }
            public Vector3 AngularVelocity { get; set; }
        }

        public Action<OutputData> OnOutput { get; set; } = null;

        OutputData _outputData = null;
        Transform _transform = null;
        Vector3 _lastPosition;
        QuaternionD _lastRotation;

        public void Initialize()
        {
            _transform = transform;
            _lastPosition = _transform.position;
            _lastRotation = new QuaternionD(_transform.rotation);
        }

        void Output()
        {
            // Position.
            var rosPosition = Ros2Utility.UnityToRos2Position(_transform.position) + MgrsPosition.Instance.Mgrs.Position;

            // Rotation.
            var rosRotation = Ros2Utility.UnityToRosRotation(_transform.rotation);

            // Angular velocity.
            var currentRotation = new QuaternionD(transform.rotation);
            var deltaRotation = currentRotation * QuaternionD.Inverse(_lastRotation);
            deltaRotation.ToAngleAxis(out var angle, out var axis);
            var angularVelocity = (1.0f / Time.deltaTime) * (float)angle * axis;
            var localAngularVelocity = transform.InverseTransformDirection(angularVelocity);
            _lastRotation = currentRotation;

            // Lineal velocity.
            var localVelocity = transform.InverseTransformDirection(_transform.position - _lastPosition) / Time.deltaTime;
            _lastPosition = transform.position;

            // TODO: Temporarily avoid NaN values. Needs investigation.
            if (float.IsNaN(localAngularVelocity.x) || float.IsNaN(localAngularVelocity.y) || float.IsNaN(localAngularVelocity.z))
                localAngularVelocity = Vector3.zero;

            // Set output data.
            _outputData.Position = rosPosition;
            _outputData.Rotation = rosRotation;
            _outputData.LinearVelocity = localVelocity;
            _outputData.AngularVelocity = localAngularVelocity;

            OnOutput?.Invoke(_outputData);
        }
    }
}