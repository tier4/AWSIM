// Copyright 2022 Robotec.ai.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;

namespace RGLUnityPlugin
{
    /// <summary>
    /// Describes mounting (pose) of a single laser of a laser array.
    /// This is a convenience structure to embed numbers that usually come from real-world manuals.
    /// Numbers are expressed in terms of Unity axes convention.
    /// </summary>
    [System.Serializable]
    public struct Laser : IEquatable<Laser>
    {
        /// <summary>
        /// Rotation around Y-axis.
        /// Positive values turn the laser right (clockwise looking from above).
        /// </summary>
        public float horizontalAngularOffsetDeg;

        /// <summary>
        /// Rotation around X-axis.
        /// Positive values turn the laser down.
        /// </summary>
        public float verticalAngularOffsetDeg;

        /// <summary>
        /// Translation along Y-axis.
        /// Positive values move the laser up.
        /// </summary>
        public float verticalLinearOffsetMm;

        /// <summary>
        /// Id of the ring.
        /// </summary>
        public int ringId;

        /// <summary>
        /// Time offset of the laser firing (in milliseconds).
        /// </summary>
        public float timeOffset;

        /// <summary>
        /// Minimum range of the laser.
        /// </summary>
        public float minRange;

        /// <summary>
        /// Maximum range of the laser.
        /// </summary>
        public float maxRange;

        //// IEquatable interface
        public bool Equals(Laser other)
        {
            return this.horizontalAngularOffsetDeg == other.horizontalAngularOffsetDeg &&
                   this.verticalAngularOffsetDeg == other.verticalAngularOffsetDeg &&
                   this.verticalLinearOffsetMm == other.verticalLinearOffsetMm &&
                   this.ringId == other.ringId &&
                   this.timeOffset == other.timeOffset &&
                   this.minRange == other.minRange &&
                   this.maxRange == other.maxRange;
        }

        public override bool Equals(object obj)
        {
            return obj is Laser equatable && Equals(equatable);
        }

        public override int GetHashCode() => (horizontalAngularOffsetDeg, verticalAngularOffsetDeg, verticalLinearOffsetMm, ringId).GetHashCode();
    }
}
