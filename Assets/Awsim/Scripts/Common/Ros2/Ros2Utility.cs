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

namespace Awsim.Common
{
    public static class Ros2Utility
    {
        /// <summary>
        /// Convert position from ROS2 to Unity.
        /// </summary>
        /// <param name="rosPosition">position by ROS2</param>
        /// <returns>position by Unity</returns>
        public static Vector3 Ros2ToUnityPosition(Vector3 rosPosition)
        {
            return new Vector3(-rosPosition.y, rosPosition.z, rosPosition.x);
        }

        /// <summary>
        /// Convert position from Unity to ROS2.
        /// </summary>
        /// <param name="unityPosition">position by Unity</param>
        /// <returns>position by ROS2</returns>
        public static Vector3 UnityToRos2Position(Vector3 unityPosition)
        {
            return new Vector3(unityPosition.z, -unityPosition.x, unityPosition.y);
        }

        /// <summary>
        /// Convert position from Unity to ROS.
        /// </summary>
        /// <param name="unityQuaternion">rotation by Unity</param>
        /// <returns>rotation by ROS</returns>
        public static Quaternion UnityToRosRotation(Quaternion unityQuaternion)
        {
            return new Quaternion(-unityQuaternion.z, unityQuaternion.x, -unityQuaternion.y, unityQuaternion.w);
        }

        /// <summary>
        /// Convert position from ROS2 to Unity.
        /// </summary>
        /// <param name="rosPosition">position by ROS2</param>
        /// <returns>position by Unity</returns>
        public static Vector3 Ros2ToUnityPosition(geometry_msgs.msg.Point rosPosition)
        {
            return new Vector3((float)-rosPosition.Y, (float)rosPosition.Z, (float)rosPosition.X);
        }

        /// <summary>
        /// Convert rotation from ROS2 to Unity.
        /// </summary>
        /// <param name="ros2Quaternion">rotation by ROS2</param>
        /// <returns>rotation by Unity</returns>
        public static Quaternion Ros2ToUnityQuaternion(Quaternion ros2Quaternion)
        {
            return new Quaternion(ros2Quaternion.y, -ros2Quaternion.z, -ros2Quaternion.x, ros2Quaternion.w);
        }


        /// <summary>
        /// Convert rotation from ROS2 to Unity.
        /// </summary>
        /// <param name="ros2Quaternion">rotation by ROS</param>
        /// <returns>rotation by Unity</returns>
        public static Quaternion Ros2ToUnityRotation(geometry_msgs.msg.Quaternion ros2Quaternion)
        {
            return new Quaternion((float)ros2Quaternion.Y, (float)-ros2Quaternion.Z, (float)-ros2Quaternion.X, (float)ros2Quaternion.W);
        }

        /// <summary>
        /// Convert scale from ROS2 to Unity.
        /// </summary>
        /// <param name="ros2Scale">scale by ROS</param>
        /// <returns>scale by Unity</returns>
        public static Vector3 Ros2ToUnityScale(Vector3 ros2Scale)
        {
            return new Vector3(ros2Scale.y, ros2Scale.z, ros2Scale.x);
        }

        /// <summary>
        /// Convert scale from Unity to ROS.
        /// </summary>
        /// <param name="unityScale">scale by Unity</param>
        /// <returns>scale by ROS</returns>
        public static Vector3 UnityToRos2Scale(Vector3 unityScale)
        {
            return new Vector3(unityScale.z, unityScale.x, unityScale.y);
        }

        /// <summary>
        /// Convert angular velocity Unity to ROS.
        /// </summary>
        /// <param name="unityAngularVelocity">angular velocity by Unity</param>
        /// <returns></returns>
        public static Vector3 UnityToRos2AngularVelocity(Vector3 unityAngularVelocity)
        {
            return UnityToRos2Position(-unityAngularVelocity);
        }
    }
}