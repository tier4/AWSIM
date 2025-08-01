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
    /// Base class for paths for interpolated iterations along an array of point.
    /// </summary>
    /// <typeparam name="TPoint"></typeparam>
    public abstract class Path<TPoint>
    {
        /// <summary>
        /// Get the length of the path.
        /// </summary>
        public float Length { get; protected set; } = 0f;

        /// <summary>
        /// Calculate a point position on the path at a distance of <paramref name="distance"/> from the start point.
        /// </summary>
        /// <param name="distance"></param>
        /// <returns>Position of the point</returns>
        public Vector3 this[float distance]
        {
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
