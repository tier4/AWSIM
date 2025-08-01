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

using ROS2;
using UnityEngine;
using Awsim.Common;

namespace Awsim.Usecase.AwsimRvizPlugins
{
    /// <summary>
    /// Feature of teleport EGO (change EGO pose) by AwsimRvizPlugins
    /// </summary>
    public class PoseTeleport
    {
        // Subscribers.
        ISubscription<geometry_msgs.msg.PoseWithCovarianceStamped> _positionSubscriber;

        Transform _transform;

        Vector3 _position = Vector3.zero;
        Quaternion _rotation = new Quaternion();
        bool _teleportFlag = false;

        /// <summary>
        /// Initialize parameter and ROS2 subscriber
        /// </summary>
        public PoseTeleport(Transform transform, string egoPositionTopic, QualityOfServiceProfile qos)
        {
            _transform = transform;

            _positionSubscriber
                = AwsimRos2Node.CreateSubscription<geometry_msgs.msg.PoseWithCovarianceStamped>(
                    egoPositionTopic, msg =>
                    {
                        Vector3 subscribed_pos = new Vector3(
                            (float)msg.Pose.Pose.Position.X,
                            (float)msg.Pose.Pose.Position.Y,
                            (float)msg.Pose.Pose.Position.Z);

                        _position = Ros2Utility.Ros2ToUnityPosition(subscribed_pos - MgrsPosition.Instance.Mgrs.Position);
                        _rotation = Ros2Utility.Ros2ToUnityRotation(msg.Pose.Pose.Orientation);
                        _teleportFlag = true;
                    }, qos);
        }

        /// <summary>
        /// Teleport EGO transform if _teleportFlag is true (i.e., coordinates of teleport destination is subscribed)
        /// </summary>
        public void OnFixedUpdate()
        {
            if (_teleportFlag && _transform != null)
            {
                Vector3 rayOrigin = new Vector3(_position.x, 1000.0f, _position.z);
                Vector3 rayDirection = Vector3.down;

                if (Physics.Raycast(rayOrigin, rayDirection, out RaycastHit hit, Mathf.Infinity))
                {
                    _transform.position = new Vector3(_position.x, hit.point.y + 1.33f, _position.z);
                    _transform.transform.rotation = _rotation;
                }
                else
                {
                    Debug.LogWarning("No mesh or collider detected on target location. Please ensure that the target location is on a mesh or collider.");
                }
            }
            _teleportFlag = false;
        }

        /// <summary>
        /// Destroy (remove) ROS2 subscriber
        /// </summary>
        public void OnDestroy()
        {
            AwsimRos2Node.RemoveSubscription<geometry_msgs.msg.PoseWithCovarianceStamped>(_positionSubscriber);
        }
    }
}