using UnityEngine;

namespace AWSIM.PointCloudMapping.Geometry {
    /// <summary>
    /// Base class for paths for interpolated iterations along an array of point.
    /// </summary>
    /// <typeparam name="TPoint"></typeparam>
    public abstract class Path<TPoint> {
        /// <summary>
        /// Get the length of the path.
        /// </summary>
        public float Length { get; protected set; } = 0f;

        /// <summary>
        /// Calculate a point position on the path at a distance of <paramref name="distance"/> from the start point.
        /// </summary>
        /// <param name="distance"></param>
        /// <returns>Position of the point</returns>
        public Vector3 this[float distance] {
            get => Point(distance);
        }

        /// <summary>
        /// Add point at the end of the path.
        /// </summary>
        /// <param name="point"></param>
        public abstract void Extend(TPoint point);

        /// <summary>
        /// Calculate a point position on the path at a distance of <paramref name="distance"/> from the start point.
        /// </summary>
        /// <param name="distance"></param>
        /// <returns>Position of point</returns>
        public abstract Vector3 Point(float distance);

    }
}
