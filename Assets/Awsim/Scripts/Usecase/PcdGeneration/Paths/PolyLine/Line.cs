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

using UnityEngine;

namespace Awsim.Usecase.PcdGeneration
{
    /// <summary>
    /// Represent a line.
    /// </summary>
    public class Line
    {
        Vector3 _point1;
        Vector3 _point2;

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
            this._point1 = point1;
            this._point2 = point2;
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
                return this._point1 + (this._point2 - this._point1) * -t;
            }

            if (t > 1)
            {
                return this._point1 * (1 - t) + this._point2;
            }

            return Vector3.Lerp(this._point1, this._point2, t);
        }
    }
}
