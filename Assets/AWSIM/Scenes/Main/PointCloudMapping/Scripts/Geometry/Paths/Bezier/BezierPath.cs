using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace AWSIM.PointCloudMapping.Geometry
{
    /// <summary>
    /// Represent a bezier curve.
    /// </summary>
    [Serializable]
    public class BezierPath : Path<BezierAnchor>
    {
        [SerializeField]
        private List<BezierAnchor> anchors = new List<BezierAnchor>();

        private List<CubicBezier> cubes;
        private List<float> cubeDistances;

        /// <summary>
        /// Get copied array of bezier anchors.
        /// </summary>
        /// <returns>Copied array of bezier anchors</returns>
        public BezierAnchor[] GetAnchors() => this.anchors.ToArray();

        /// <summary>
        /// Initialize <see cref="BezierPath"/>
        /// </summary>
        /// <param name="anchorPosition"></param>
        /// <param name="handleVector"></param>
        /// <param name="oneSided"></param>
        public BezierPath(Vector3 anchorPosition, Vector3 handleVector, bool oneSided = false)
        {
            var controlPosition1 = oneSided ? anchorPosition : anchorPosition - handleVector;
            var controlPosition2 = anchorPosition + handleVector;
            var anchor = new BezierAnchor(anchorPosition, controlPosition1, controlPosition2);
            this.anchors.Add(anchor);
        }

        /// <summary>
        /// Initialize <see cref="BezierPath"/>
        /// </summary>
        /// <param name="anchor"></param>
        public BezierPath(BezierAnchor anchor)
        {
            Extend(anchor);
        }

        /// <summary>
        /// Add anchor point at the end of the path.
        /// </summary>
        /// <param name="anchorPosition"></param>
        /// <param name="handleVector"></param>
        /// <param name="oneSided"></param>
        public void Extend(Vector3 anchorPosition, Vector3 handleVector, bool oneSided = false)
        {
            var controlPosition1 = anchorPosition - handleVector;
            var controlPosition2 = oneSided ? anchorPosition : anchorPosition + handleVector;
            var anchor = new BezierAnchor(anchorPosition, controlPosition1, controlPosition2);
            Extend(anchor);
        }

        /// <summary>
        /// Add anchor point at the end of the path.
        /// </summary>
        /// <param name="anchor"></param>
        public override void Extend(BezierAnchor anchor)
        {
            if (!IsInitialized())
            {
                SetupCubes();
            }
            this.anchors.Add(anchor);
            if (this.anchors.Count == 1)
            {
                return;
            }
            var anchor1 = this.anchors[this.anchors.Count - 2];
            var cube = new CubicBezier(anchor1, anchor);
            this.cubes.Add(cube);
            this.cubeDistances.Add(this.Length);
            this.Length += cube.Length;
        }

        /// <summary>
        /// Calculate a tangent vector on the path at a distance of <paramref name="distance"/> from the start point.
        /// </summary>
        /// <param name="distance"></param>
        /// <returns></returns>
        public Vector3 Tangent(float distance)
        {
            if (!IsInitialized())
            {
                SetupCubes();
            }
            var index = GetIndexOfNearestSmall(distance);
            return this.cubes[index].Tangent(distance - this.cubeDistances[index]);
        }

        public override Vector3 Point(float distance)
        {
            return Point(distance);
        }

        /// <summary>
        /// Calculate a pose with a position and a tangent direction on the path at a distance of <paramref name="distance"/> from the start point.
        /// </summary>
        /// <param name="distance"></param>
        /// <returns></returns>
        public Pose TangentPose(float distance)
        {
            if (!IsInitialized())
            {
                SetupCubes();
            }
            var index = GetIndexOfNearestSmall(distance);
            return this.cubes[index].TangentPose(distance - this.cubeDistances[index]);
        }

        private bool IsInitialized() => this.cubes != null && this.cubeDistances != null;

        private void SetupCubes()
        {
            this.Length = 0f;
            this.cubes = new List<CubicBezier>();
            this.cubeDistances = new List<float>();
            if (this.anchors.Count == 0)
            {
                return;
            }
            var prevAnchor = this.anchors.First();
            foreach (var anchor in this.anchors)
            {
                if (prevAnchor.Position == anchor.Position)
                {
                    continue;
                }
                var cube = new CubicBezier(prevAnchor, anchor);
                this.cubes.Add(cube);
                this.cubeDistances.Add(this.Length);
                this.Length += cube.Length;
                prevAnchor = anchor;
            }
            return;
        }

        private int GetIndexOfNearestSmall(float distance)
        {
            var cubeCount = this.cubeDistances.Count;
            var startIndex = Mathf.FloorToInt((cubeCount - 1) * distance / this.Length);
            if (this.cubeDistances[startIndex] <= distance)
            {
                for (int i = startIndex + 1; i < cubeCount; ++i)
                {
                    if (this.cubeDistances[i] > distance)
                    {
                        return i - 1;
                    }
                }
                return cubeCount - 1;
            }
            else
            {
                for (int i = startIndex - 1; i > 0; --i)
                {
                    if (this.cubeDistances[i] <= distance)
                    {
                        return i;
                    }
                }
                return 0;
            }
        }
    }
}
