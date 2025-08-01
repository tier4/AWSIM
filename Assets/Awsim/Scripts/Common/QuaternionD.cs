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

namespace Awsim.Common
{
    /// <summary>
    /// A structure that defines a double precision based quaternion,
    /// which is a mathematical tool to represent orientations and rotations.
    /// </summary>
    public struct QuaternionD
    {
        /// <summary>
        /// The x component of the Quaternion.
        /// </summary>
        public double X;
        /// <summary>
        /// The y component of the Quaternion.
        /// </summary>
        public double Y;
        /// <summary>
        /// The z component of the Quaternion.
        /// </summary>
        public double Z;
        /// <summary>
        /// The w component of the Quaternion.
        /// </summary>
        public double W;

        /// <summary>
        /// Constructs a quaternion from the given components.
        /// </summary>
        /// <param name="x">The x component of the Quaternion.</param>
        /// <param name="y">The y component of the Quaternion.</param>
        /// <param name="z">The z component of the Quaternion.</param>
        /// <param name="w">The w component of the Quaternion.</param>
        public QuaternionD(double x, double y, double z, double w)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
            this.W = w;
        }

        /// <summary>
        /// Constructs a double precision quaternion from Unity quaternion.
        /// </summary>
        /// <param name="q">The single precision quaternion.</param>
        public QuaternionD(UnityEngine.Quaternion q)
        {
            this.X = (double)q.x;
            this.Y = (double)q.y;
            this.Z = (double)q.z;
            this.W = (double)q.w;
        }

        /// <summary>
        /// Set the components of the quaternion.
        /// </summary>
        /// <param name="x">The x component of the Quaternion.</param>
        /// <param name="y">The y component of the Quaternion.</param>
        /// <param name="z">The z component of the Quaternion.</param>
        /// <param name="w">The w component of the Quaternion.</param>
        public void Set(double x, double y, double z, double w)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
            this.W = w;
        }

        /// <summary>
        /// Returns a quaternion that represents a neutral operation (no rotation). 
        /// </summary>
        public static QuaternionD Identity
        {
            get { return new QuaternionD(0, 0, 0, 1); }
        }

        /// <summary>
        /// Calculates the squared length of the quaternion.
        /// </summary>
        /// <returns>The squared length of the quaternion.</returns>
        public double SquaredLength()
        {
            return X * X + Y * Y + Z * Z + W * W;
        }

        /// <summary>
        /// Calculates the length of the quaternion.
        /// </summary>
        /// <returns>The length of the quaternion.</returns>
        public double Length()
        {
            var squaredLength = this.SquaredLength();
            return Math.Sqrt(squaredLength);
        }

        /// <summary>
        /// Normalizes the quaternion by division of each component by the length.
        /// </summary>
        /// <param name="q">The quaternion that will be normalized.</param>
        /// <returns>The normalized Quaternion.</returns>
        public QuaternionD Normalize()
        {
            double invNorm = 1.0 / this.Length();

            QuaternionD q;
            q.X = X * invNorm;
            q.Y = Y * invNorm;
            q.Z = Z * invNorm;
            q.W = W * invNorm;

            return q;
        }

        /// <summary>
        /// Inverse of the quaternion.
        /// </summary>
        /// <param name="q">The quaternion that will be inverted.</param>
        /// <returns>The inverted Quaternion.</returns>
        public static QuaternionD Inverse(QuaternionD q)
        {
            QuaternionD result;

            double squaredLength = q.SquaredLength();
            double invNorm = 1.0 / squaredLength;

            result.X = -q.X * invNorm;
            result.Y = -q.Y * invNorm;
            result.Z = -q.Z * invNorm;
            result.W = q.W * invNorm;

            return result;
        }

        /// <summary>
        /// Returns conjugate of the quaternion.
        /// </summary>
        /// <param name="q">The input quaternion.</param>
        /// <returns>A quaternion that is the conjugate of the input one.</returns>
        public static QuaternionD Conjugate(QuaternionD q)
        {
            QuaternionD result;

            result.X = -q.X;
            result.Y = -q.Y;
            result.Z = -q.Z;
            result.W = q.W;

            return result;
        }

        /// <summary>
        /// Creates a quaternion from the given RPY (yaw, pitch, roll) angles.
        /// </summary>
        /// <param name="roll">The roll angle in radians.</param>
        /// <param name="pitch">The pitch angle in radians.</param>
        /// <param name="yaw">The yaw angle in radians.</param>
        /// <returns></returns>
        public static QuaternionD CreateFromRPY(double roll, double pitch, double yaw)
        {
            var sr = Math.Sin(roll * 0.5);
            var cr = Math.Cos(roll * 0.5);

            var sp = Math.Sin(pitch * 0.5);
            var cp = Math.Cos(pitch * 0.5);

            var sy = Math.Sin(yaw * 0.5);
            var cy = Math.Cos(yaw * 0.5);

            QuaternionD result;

            result.X = cy * sp * cr + sy * cp * sr;
            result.Y = sy * cp * cr - cy * sp * sr;
            result.Z = cy * cp * sr - sy * sp * cr;
            result.W = cy * cp * cr + sy * sp * sr;

            return result;
        }

        /// <summary>
        /// Creates the quaternion from an axis and an angle.
        /// </summary>
        /// <param name="angle">The angle in radians, to rotate around the axis.</param>
        /// <param name="axis">The axis to rotate around.</param>
        /// <returns>The quaternion.</returns>
        public static QuaternionD FromAngleAxis(double angle, UnityEngine.Vector3 axis)
        {
            QuaternionD result;

            double halfAngle = angle * 0.5;
            double s = Math.Sin(halfAngle);

            result.X = axis.x * s;
            result.Y = axis.y * s;
            result.Z = axis.z * s;
            result.W = Math.Cos(halfAngle);

            return result;
        }

        /// <summary>
        /// Creates the quaternion from an axis and an angle.
        /// </summary>
        /// <param name="angle">The angle in radians, to rotate around the axis.</param>
        /// <param name="axis">The axis to rotate around.</param>
        /// <returns>The quaternion.</returns>
        public void ToAngleAxis(out double angle, out UnityEngine.Vector3 axis)
        {
            this = this.Normalize();
            angle = Math.Acos(W) * 2;

            double sin = Math.Asin(angle / 2.0);

            axis = new UnityEngine.Vector3();
            axis.x = (float)(X / sin);
            axis.y = (float)(Y / sin);
            axis.z = (float)(Z / sin);
        }

        /// <summary>
        /// Adds two quaternions.
        /// </summary>
        /// <param name="q1">The first quaternion.</param>
        /// <param name="q2">The second quaternion.</param>
        /// <returns>The result of adding the quaternions.</returns>
        public static QuaternionD operator +(QuaternionD q1, QuaternionD q2)
        {
            QuaternionD result;

            result.X = q1.X + q2.X;
            result.Y = q1.Y + q2.Y;
            result.Z = q1.Z + q2.Z;
            result.W = q1.W + q2.W;

            return result;
        }

        /// <summary>
        /// Subtracts one quaternion from the other.
        /// </summary>
        /// <param name="q1">The first quaternion.</param>
        /// <param name="q2">The second quaternion.</param>
        /// <returns>The result of the substraction of quaternions.</returns>
        public static QuaternionD operator -(QuaternionD q1, QuaternionD q2)
        {
            QuaternionD result;

            result.X = q1.X - q2.X;
            result.Y = q1.Y - q2.Y;
            result.Z = q1.Z - q2.Z;
            result.W = q1.W - q2.W;

            return result;
        }

        /// <summary>
        /// Multiplies two quaternions.
        /// </summary>
        /// <param name="q1">The quaternion on the left side of the multiplication.</param>
        /// <param name="q2">The quaternion on the right side of the multiplication.</param>
        /// <returns>The result of quaternions multiplication multiplication.</returns>
        public static QuaternionD operator *(QuaternionD q1, QuaternionD q2)
        {
            QuaternionD result;

            double cx = q1.Y * q2.Z - q1.Z * q2.Y;
            double cy = q1.Z * q2.X - q1.X * q2.Z;
            double cz = q1.X * q2.Y - q1.Y * q2.X;

            double dot = q1.X * q2.X + q1.Y * q2.Y + q1.Z * q2.Z;

            result.X = q1.X * q2.W + q2.X * q1.W + cx;
            result.Y = q1.Y * q2.W + q2.Y * q1.W + cy;
            result.Z = q1.Z * q2.W + q2.Z * q1.W + cz;
            result.W = q1.W * q2.W - dot;

            return result;
        }

        /// <summary>
        /// Checks if two quaternions are equal.
        /// </summary>
        /// <param name="q1">The first quaternion.</param>
        /// <param name="q2">The second quaternion.</param>
        /// <returns>True if the quaternions are equal. Otherwise returns false.</returns>
        public static bool operator ==(QuaternionD q1, QuaternionD q2)
        {
            return (q1.X == q2.X && q1.Y == q2.Y && q1.Z == q2.Z && q1.W == q2.W);
        }

        /// <summary>
        /// Checks if two quaternions are not equal.
        /// </summary>
        /// <param name="q1">The first quaternion.</param>
        /// <param name="q2">The second quaternion.</param>
        /// <returns>True if the quaternions are not equal. Otherwise returns false.</returns>
        public static bool operator !=(QuaternionD q1, QuaternionD q2)
        {
            return (q1.X != q2.X || q1.Y != q2.Y || q1.Z != q2.Z || q1.W != q2.W);
        }

        /// <summary>
        /// Checks whether the given Object is equal to this quaternion.
        /// </summary>
        /// <param name="q1">The Object to compare against.</param>
        /// <returns>True if the Object is equal to this Quaternion; False otherwise.</returns>
        public override bool Equals(object q)
        {
            if (q is QuaternionD)
            {
                return (QuaternionD)q == this;
            }

            return false;
        }

        /// <summary>
        /// Calculates the hash code for this instance.
        /// </summary>
        /// <returns>The hash code.</returns>
        public override int GetHashCode()
        {
            return X.GetHashCode() + Y.GetHashCode() + Z.GetHashCode() + W.GetHashCode();
        }

        /// <summary>
        /// Converts a quaternion to string.
        /// </summary>
        /// <returns>The string represented quaternion.</returns>
        public override string ToString()
        {
            return String.Format("{{x:{0} y:{1} z:{2} w:{3}}}", X.ToString(), Y.ToString(), Z.ToString(), W.ToString());
        }
    }
}