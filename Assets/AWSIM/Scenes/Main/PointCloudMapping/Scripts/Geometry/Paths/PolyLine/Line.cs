using UnityEngine;

namespace AWSIM.PointCloudMapping.Geometry
{
    /// <summary>
    /// Represent a line.
    /// </summary>
    public class Line
    {
        private Vector3 point1;
        private Vector3 point2;

        /// <summary>
        /// Get the length of the line.
        /// </summary>
        public float Length { get; private set; } = 0f;

        /// <summary>
        /// Initialize <see cref="Line"/>
        /// </summary>
        /// <param name="point1"></param>
        /// <param name="point2"></param>
        public Line(Vector3 point1, Vector3 point2)
        {
            this.point1 = point1;
            this.point2 = point2;
            this.Length = Vector3.Distance(point1, point2);
        }

        /// <summary>
        /// Calculate position of a point on the line at a distance of <paramref name="distance"/>.
        /// </summary>
        /// <param name="distance"></param>
        /// <returns></returns>
        public Vector3 Point(float distance)
        {
            float t = distance / this.Length;

            if (t < 0)
            {
                return this.point1 + (this.point2 - this.point1) * -t;
            }

            if (t > 1)
            {
                return this.point1 * (1 - t) + this.point2;
            }

            return Vector3.Lerp(this.point1, this.point2, t);
        }
    }
}
