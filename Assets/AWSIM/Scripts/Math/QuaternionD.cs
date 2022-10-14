using System;

namespace AWSIM
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
        public double x;
        /// <summary>
        /// The y component of the Quaternion.
        /// </summary>
        public double y;
        /// <summary>
        /// The z component of the Quaternion.
        /// </summary>
        public double z;
        /// <summary>
        /// The w component of the Quaternion.
        /// </summary>
        public double w;

        /// <summary>
        /// Constructs a quaternion from the given components.
        /// </summary>
        /// <param name="x">The x component of the Quaternion.</param>
        /// <param name="y">The y component of the Quaternion.</param>
        /// <param name="z">The z component of the Quaternion.</param>
        /// <param name="w">The w component of the Quaternion.</param>
        public QuaternionD(double x, double y, double z, double w)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }

        /// <summary>
        /// Constructs a double precision quaternion from Unity quaternion.
        /// </summary>
        /// <param name="q">The single precision quaternion.</param>
        public QuaternionD(UnityEngine.Quaternion q)
        {
            this.x = (double)q.x;
            this.y = (double)q.y;
            this.z = (double)q.z;
            this.w = (double)q.w;
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
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
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
            return x * x + y * y + z * z + w * w;
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
            q.x = x * invNorm;
            q.y = y * invNorm;
            q.z = z * invNorm;
            q.w = w * invNorm;
 
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
 
            result.x = -q.x * invNorm;
            result.y = -q.y * invNorm;
            result.z = -q.z * invNorm;
            result.w = q.w * invNorm;
 
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
 
            result.x = -q.x;
            result.y = -q.y;
            result.z = -q.z;
            result.w = q.w;
 
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
 
            result.x = cy * sp * cr + sy * cp * sr;
            result.y = sy * cp * cr - cy * sp * sr;
            result.z = cy * cp * sr - sy * sp * cr;
            result.w = cy * cp * cr + sy * sp * sr;
 
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
 
            result.x = axis.x * s;
            result.y = axis.y * s;
            result.z = axis.z * s;
            result.w = Math.Cos(halfAngle);
 
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
            angle = Math.Acos(w) * 2;

            double sin = Math.Asin(angle / 2.0);

            axis = new UnityEngine.Vector3();
            axis.x = (float) (x / sin);
            axis.y = (float) (y / sin);
            axis.z = (float) (z / sin);
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
 
            result.x = q1.x + q2.x;
            result.y = q1.y + q2.y;
            result.z = q1.z + q2.z;
            result.w = q1.w + q2.w;
 
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
 
            result.x = q1.x - q2.x;
            result.y = q1.y - q2.y;
            result.z = q1.z - q2.z;
            result.w = q1.w - q2.w;
 
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

            double cx = q1.y * q2.z - q1.z * q2.y;
            double cy = q1.z * q2.x - q1.x * q2.z;
            double cz = q1.x * q2.y - q1.y * q2.x;
 
            double dot = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;

            result.x = q1.x * q2.w + q2.x * q1.w + cx;
            result.y = q1.y * q2.w + q2.y * q1.w + cy;
            result.z = q1.z * q2.w + q2.z * q1.w + cz;
            result.w = q1.w * q2.w - dot;

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
            return (q1.x == q2.x && q1.y == q2.y && q1.z == q2.z && q1.w == q2.w);
        }

        /// <summary>
        /// Checks if two quaternions are not equal.
        /// </summary>
        /// <param name="q1">The first quaternion.</param>
        /// <param name="q2">The second quaternion.</param>
        /// <returns>True if the quaternions are not equal. Otherwise returns false.</returns>
        public static bool operator !=(QuaternionD q1, QuaternionD q2)
        {
            return (q1.x != q2.x || q1.y != q2.y || q1.z != q2.z || q1.w != q2.w);
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
            return x.GetHashCode() + y.GetHashCode() + z.GetHashCode() + w.GetHashCode();
        }

        /// <summary>
        /// Converts a quaternion to string.
        /// </summary>
        /// <returns>The string represented quaternion.</returns>
        public override string ToString()
        {
            return String.Format("{{x:{0} y:{1} z:{2} w:{3}}}", x.ToString(), y.ToString(), z.ToString(), w.ToString());
        }
    }
}