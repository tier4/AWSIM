using System;
using UnityEngine;

namespace AWSIM.PointCloudMapping.Geometry
{
    /// <summary>
    /// Provide information of a anchor point of bezier.
    /// </summary>
    [Serializable]
    public struct BezierAnchor
    {
        /// <summary>
        /// Position of the anchor point.
        /// </summary>
        public Vector3 Position;
        /// <summary>
        /// Position of the control point on the side of start point.
        /// </summary>
        public Vector3 Control1;
        /// <summary>
        /// Position of the control point on the side of end point.
        /// </summary>
        public Vector3 Control2;

        /// <summary>
        /// Initialize <see cref="BezierAnchor"/>
        /// </summary>
        /// <param name="position"></param>
        /// <param name="control1"></param>
        /// <param name="control2"></param>
        public BezierAnchor(Vector3 position, Vector3 control1, Vector3 control2)
        {
            this.Position = position;
            this.Control1 = control1;
            this.Control2 = control2;
        }
    }
}
