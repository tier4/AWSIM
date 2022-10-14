using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// ROS2 utility static class.
    /// </summary>
    public static class ROS2Utility
    {
        /// <summary>
        /// Convert position from ROS to Unity.
        /// </summary>
        /// <param name="rosPosition">position by ROS</param>
        /// <returns>position by Unity</returns>
        public static Vector3 RosToUnityPosition(Vector3 rosPosition)
        {
            return new Vector3(-rosPosition.y, rosPosition.z, rosPosition.x);
        }

        /// <summary>
        /// Convert position from Unity to ROS.
        /// </summary>
        /// <param name="unityPosition">position by Unity</param>
        /// <returns>position by ROS</returns>
        public static Vector3 UnityToRosPosition(Vector3 unityPosition)
        {
            return new Vector3(unityPosition.z, -unityPosition.x, unityPosition.y);
        }

        /// <summary>
        /// Convert position from ROS to Unity.
        /// </summary>
        /// <param name="rosQuaternion">rotation by ROS</param>
        /// <returns>rotation by Unity</returns>
        public static Quaternion RosToUnityRotation(Quaternion rosQuaternion)
        {
            return new Quaternion(rosQuaternion.y, -rosQuaternion.z, -rosQuaternion.x, rosQuaternion.w);
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
        /// Convert scale from ROS to Unity.
        /// </summary>
        /// <param name="rosScale">scale by ROS</param>
        /// <returns>scale by Unity</returns>
        public static Vector3 RosToUnityScale(Vector3 rosScale)
        {
            return new Vector3(rosScale.y, rosScale.z, rosScale.x);
        }

        /// <summary>
        /// Convert scale from Unity to ROS.
        /// </summary>
        /// <param name="unityScale">scale by Unity</param>
        /// <returns>scale by ROS</returns>
        public static Vector3 UnityToRosScale(Vector3 unityScale)
        {
            return new Vector3(unityScale.z, unityScale.x, unityScale.y);
        }

        /// <summary>
        /// Convert angular velocity Unity to ROS.
        /// </summary>
        /// <param name="unityAngularVelocity">angular velocity by Unity</param>
        /// <returns></returns>
        public static Vector3 UnityToRosAngularVelocity(Vector3 unityAngularVelocity)
        {
            return UnityToRosPosition(-unityAngularVelocity);
        }
    }
}