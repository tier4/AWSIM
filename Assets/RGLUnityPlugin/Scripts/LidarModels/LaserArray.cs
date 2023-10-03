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
using System.Linq;
using UnityEngine;

namespace RGLUnityPlugin
{
    /// <summary>
    /// Describes a LiDAR's array of lasers.
    /// </summary>
    [System.Serializable]
    public struct LaserArray
    {
        //             |
        //  .---------------------.
        //  |                     |
        //  |          x - - - -  |  - - - -
        //  |          |          |        ^
        //  |                     |        | this distance
        //  |                     |        v
        //  ._____________________.  - - - -
        //             |
        // Note: It is not always lay on the axis of symmetry of the device (e.g., hybrid solid-state LIDARs)
        // This offset is not considered when generating laser poses. To be applied manually when setting the pose of the lidar (in RGL).
        public Vector3 centerOfMeasurementLinearOffsetMm;

        //             |           /
        //  .--------------------/.
        //  |                  /  |
        //  |          x - - - ---|-----------
        //  |          |       \  |
        //  |                  .\ |
        //  |                  . \|
        //  .__________________.__\
        //             |       .   \
        //             .       .
        //             .<----->.
        //           this distance
        // Distance from the sensor center to the focal point where all laser beams intersect.
        public float focalDistanceMm;

        /// <summary>
        /// List of lasers constituting this array.
        /// Note: Set minRange and maxRange of the lasers if ranges for them differ. If not, range can be specified in the LidarConfiguration. 
        /// </summary>
        public Laser[] lasers;

        /// <summary>
        /// Returns poses of lasers relative to the attached game object origin.
        /// </summary>
        public Matrix4x4[] GetLaserPoses()
        {
            // Workaround due to the following C# constraint:
            // Anonymous methods, lambda expressions, local functions and query expressions inside structs cannot access
            // instance members of 'this'. Consider copying 'this' to a local variable outside the anonymous method, lambda
            // expression, local function or query expression and using the local instead.
            LaserArray self = this;
            return lasers.Select(
                laser => Matrix4x4.Translate(Vector3.up * mmToMeters(laser.verticalLinearOffsetMm) +
                                             Vector3.forward *
                                             mmToMeters(self.focalDistanceMm))
                         * Matrix4x4.Rotate(Quaternion.Euler(laser.verticalAngularOffsetDeg,
                             laser.horizontalAngularOffsetDeg,
                             0.0f))).ToArray();
        }

        public Vector2[] GetLaserRanges()
        {
            return lasers.Select(laser => new Vector2(laser.minRange, laser.maxRange)).ToArray();
        }

        public int[] GetLaserRingIds()
        {
            return lasers.Select(laser => laser.ringId).ToArray();
        }

        /// <summary>
        /// Generates LaserArray with uniformly distributed vertical angular offsets.
        /// Note: angles follow Unity convention, looking above the horizon is negative angle! 
        /// </summary>
        public static LaserArray Uniform(float minVAngle, float maxVAngle, int vIncrements)
        {
            if (!(minVAngle <= maxVAngle))
            {
                throw new ArgumentOutOfRangeException(nameof(minVAngle),
                    "Minimum angle must be lower or equal to maximum angle");
            }

            if (!(vIncrements > 0))
            {
                throw new ArgumentOutOfRangeException(nameof(vIncrements),
                    "Number of vertical steps must be positive");
            }

            Laser[] lasers = new Laser[vIncrements];

            float vertIncrAngle = (maxVAngle - minVAngle) / vIncrements;
            for (int incr = 0; incr < vIncrements; incr++)
            {
                float angle = minVAngle + incr * vertIncrAngle;
                lasers[incr] = new Laser {verticalAngularOffsetDeg = angle, ringId = incr + 1};
            }

            return new LaserArray()
            {
                centerOfMeasurementLinearOffsetMm = new Vector3(0.0f, 0.0f, 0.0f),
                focalDistanceMm = 0.0f,
                lasers = lasers
            };
        }

        private static float mmToMeters(float mm)
        {
            return mm / 1000.0f;
        }
    }
}