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

namespace Awsim.Entity
{
    /// <summary>
    /// NPC pedestrian that is controlled in the scenario.
    /// </summary>
    [RequireComponent(typeof(Rigidbody), typeof(Animator))]
    public class Pedestrian : MonoBehaviour
    {

        const string _moveSpeedProperty = "moveSpeed";
        const string _rotateSpeedProperty = "rotateSpeed";

        /// <summary>
        /// Get the reference point of the pedestrian.
        /// </summary>
        public Transform ReferencePoint => _referencePoint;

        public event Action<Collider> OnTriggerEnterAction;

        [SerializeField] Rigidbody _rigidbody;
        [SerializeField] Transform _referencePoint;
        [SerializeField, Tooltip(
             "Pedestrian Animator component.\n" +
             "The animator should have the following float parameters for proper transitions.\n" +
             "    moveSpeed: Pedestrian movement speed in m/s\n" +
             "    rotateSpeed: Pedestrian rotation speed in rad/s")]
        Animator _animator;

        [Header("Ground follow parameters")]
        [SerializeField, Tooltip("Ray-cast max distance for locating the ground.")]
        float _rayCastMaxDistance = 100f;
        [SerializeField, Tooltip("Upward offset of the ray-cast origin from the GameObject local origin for locating the ground.")]
        float _rayCastOriginOffset = 1f;
        public Pose PoseInput { get; set; } = new Pose();

        public void Initialize()
        {
            PoseInput = new Pose(_rigidbody.position, _rigidbody.rotation);
        }

        public void OnUpdate()
        {
            // Switch animation based on movement speed (m/s).
            var speed2D = new Vector2(_rigidbody.linearVelocity.x, _rigidbody.linearVelocity.z).magnitude;
            _animator.SetFloat(_moveSpeedProperty, speed2D);

            // Switch animation based on rotation speed (rad/s).
            _animator.SetFloat(_rotateSpeedProperty, _rigidbody.angularVelocity.magnitude);
        }

        public void OnFixedUpdate()
        {
            ApplyPoseInput(PoseInput);
        }

        void ApplyPoseInput(Pose pose)
        {
            var rootPosition = ConvertReferenceToRoot(pose.position);
            var groundPosition = FollowGround(rootPosition);
            _rigidbody.MovePosition(groundPosition);

            var rootRotation = ConvertReferenceToRoot(pose.rotation);
            _rigidbody.MoveRotation(rootRotation);
        }

        void OnTriggerEnter(Collider other)
        {
            OnTriggerEnterAction?.Invoke(other);
        }

        Vector3 FollowGround(Vector3 position)
        {
            var origin = position + Vector3.up * _rayCastOriginOffset;
            var groundExists = Physics.Raycast(origin, Vector3.down, out var hitInfo, _rayCastMaxDistance);
            return groundExists ? hitInfo.point : position;
        }

        Quaternion ConvertReferenceToRoot(Quaternion rotation)
        {
            return rotation * Quaternion.Inverse(_referencePoint.rotation) * transform.rotation;
        }

        Vector3 ConvertReferenceToRoot(Vector3 position)
        {
            return position - _referencePoint.position + transform.position;
        }
    }
}
