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

using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Awsim.Usecase.PcdGeneration
{
    /// <summary>
    /// Represent a path of polyline.
    /// </summary>
    public class PolyLinePath : Path<Vector3>
    {
        List<Vector3> _points = new List<Vector3>();
        SortedList<float, Line> _lines = new SortedList<float, Line>();

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
            this._points.Add(point);
            if (this._points.Count == 1)
            {
                return;
            }
            var point1 = this._points[this._points.Count - 2];
            var line = new Line(point1, point);
            if (line.Length < 0.001f)
            {
                return;
            }
            this._lines.Add(this.Length, line);
            this.Length += line.Length;
        }

        public override Vector3 Point(float distance)
        {
            var key = NearestSmallKey(distance);
            return this._lines[key].Point(distance - key);
        }

        private float NearestSmallKey(float distance)
        {
            var prevLine = this._lines.First();
            foreach (var line in this._lines)
            {
                if (line.Key > distance)
                {
                    return prevLine.Key;
                }
                prevLine = line;
            }
            return this._lines.Keys.Last();
        }
    }
}
