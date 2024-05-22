using System;
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
        /// Create Ros Quaternion from euler rotations
        /// </summary>
        /// <param name="roll">roll euler rotation [rad]</param>
        /// <param name="pitch">pitch euler rotation [rad]</param>
        /// <param name="yaw">yaw euler rotation [rad]</param>
        /// <returns>position by Unity</returns>
        public static geometry_msgs.msg.Quaternion RosQuaternionFromEuler(double roll, double pitch, double yaw)
        {
            double cy = Math.Cos(yaw * 0.5f);
            double sy = Math.Sin(yaw * 0.5f);
            double cp = Math.Cos(pitch * 0.5f);
            double sp = Math.Sin(pitch * 0.5f);
            double cr = Math.Cos(roll * 0.5f);
            double sr = Math.Sin(roll * 0.5f);

            geometry_msgs.msg.Quaternion q = new geometry_msgs.msg.Quaternion();
            q.W = cy * cp * cr + sy * sp * sr;
            q.X = cy * cp * sr - sy * sp * cr;
            q.Y = sy * cp * sr + cy * sp * cr;
            q.Z = sy * cp * cr - cy * sp * sr;
            return q;
        }

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
        /// <param name="rosPosition">position by ROS</param>
        /// <returns>position by Unity</returns>
        public static Vector3 RosToUnityPosition(geometry_msgs.msg.Point rosPosition)
        {
            return new Vector3((float)-rosPosition.Y, (float)rosPosition.Z, (float)rosPosition.X);
        }

        /// <summary>
        /// Convert position from ROS (MGRS) to Unity (word coordinate system).
        /// </summary>
        /// <param name="rosPosition">position by ROS in MGRS</param>
        /// <returns>position by Unity in Unity WORLD</returns>
        public static Vector3 RosMGRSToUnityPosition(geometry_msgs.msg.Point rosPosition)
        {
            var offset = Environment.Instance.MgrsOffsetPosition;
            return new Vector3(-(float)(rosPosition.Y - offset.y),
                               (float)(rosPosition.Z - offset.z),
                               (float)(rosPosition.X - offset.x));
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
        /// Convert rotation from ROS to Unity.
        /// </summary>
        /// <param name="rosQuaternion">rotation by ROS</param>
        /// <returns>rotation by Unity</returns>
        public static Quaternion RosToUnityRotation(geometry_msgs.msg.Quaternion rosQuaternion)
        {
            return new Quaternion((float)rosQuaternion.Y, (float)-rosQuaternion.Z, (float)-rosQuaternion.X, (float)rosQuaternion.W);
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
