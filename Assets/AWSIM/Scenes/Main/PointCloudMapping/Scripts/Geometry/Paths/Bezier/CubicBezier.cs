using UnityEngine;

namespace AWSIM.PointCloudMapping.Geometry
{
    /// <summary>
    /// Provide information of a cubic bezier.
    /// </summary>
    internal struct CubicBezier
    {
        public float Length;
        public Vector3 p0;
        public Vector3 p1;
        public Vector3 p2;
        public Vector3 p3;

        public CubicBezier(BezierAnchor anchor1, BezierAnchor anchor2)
        {
            this.p0 = anchor1.Position;
            this.p1 = anchor1.Control2;
            this.p2 = anchor2.Control1;
            this.p3 = anchor2.Position;
            this.Length = CalculateLength(this.p0, this.p1, this.p2, this.p3);
        }

        public Pose TangentPose(float distance)
        {
            var vertex = Point(distance);
            var tangent = Tangent(distance);
            var normal = Normal(tangent);
            return new Pose
            {
                position = vertex,
                rotation = Quaternion.LookRotation(tangent, normal)
            };
        }

        public Vector3 Point(float distance)
        {
            var t = distance / this.Length;
            var u = 1 - t;
            var w0 = u * u * u;
            var w1 = 3 * t * u * u;
            var w2 = 3 * t * t * u;
            var w3 = t * t * t;
            return w0 * this.p0 + w1 * this.p1 + w2 * this.p2 + w3 * this.p3;
        }

        public Vector3 Tangent(float distance)
        {
            var t = distance / this.Length;
            var u = 1 - t;
            var w0 = -u * u;
            var w1 = 3 * t * t - 4 * t + 1;
            var w2 = -3 * t * t + 2 * t;
            var w3 = t * t;
            return (w0 * this.p0 + w1 * this.p1 + w2 * this.p2 + w3 * this.p3).normalized;
        }

        public Vector3 Normal(float distance)
        {
            return Normal(Tangent(distance));
        }

        private Vector3 Normal(Vector3 tangent)
        {
            return Vector3.Cross(tangent, Vector3.Cross(Vector3.up, tangent)).normalized;
        }

        public void UpdateLength()
        {
            this.Length = CalculateLength(this.p0, this.p1, this.p2, this.p3);
        }

        private static float CalculateLength(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
        {
            float chordLength = Vector3.Distance(p3, p0);
            float controlNetLength = Vector3.Distance(p0, p1) +
                                     Vector3.Distance(p2, p1) +
                                     Vector3.Distance(p3, p2);
            return (chordLength + controlNetLength) / 2f;
        }
    }
}
