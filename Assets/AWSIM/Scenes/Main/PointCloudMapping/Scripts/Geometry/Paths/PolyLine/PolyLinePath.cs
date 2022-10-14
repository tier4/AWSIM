using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace AWSIM.PointCloudMapping.Geometry
{
    /// <summary>
    /// Represent a path of polyline.
    /// </summary>
    public class PolyLinePath : Path<Vector3>
    {
        private List<Vector3> points = new List<Vector3>();
        private SortedList<float, Line> lines = new SortedList<float, Line>();

        /// <summary>
        /// Initialize <see cref="PolyLinePath"/>.
        /// </summary>
        public PolyLinePath() { }

        /// <summary>
        /// Initialize <see cref="PolyLinePath"/>.
        /// </summary>
        /// <param name="points"></param>
        public PolyLinePath(Vector3[] points)
        {
            foreach (var point in points)
            {
                Extend(point);
            }
        }

        public override void Extend(Vector3 point)
        {
            this.points.Add(point);
            if (this.points.Count == 1)
            {
                return;
            }
            var point1 = this.points[this.points.Count - 2];
            var line = new Line(point1, point);
            if (line.Length < 0.001f)
            {
                return;
            }
            this.lines.Add(this.Length, line);
            this.Length += line.Length;
        }

        public override Vector3 Point(float distance)
        {
            var key = NearestSmallKey(distance);
            return this.lines[key].Point(distance - key);
        }

        private float NearestSmallKey(float distance)
        {
            var prevLine = this.lines.First();
            foreach (var line in this.lines)
            {
                if (line.Key > distance)
                {
                    return prevLine.Key;
                }
                prevLine = line;
            }
            return this.lines.Keys.Last();
        }
    }
}
