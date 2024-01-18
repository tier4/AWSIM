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
using System.Collections.Generic;
using UnityEngine;

namespace RGLUnityPlugin
{
    /// <summary>
    /// Base class for lidar configuration. It contains:
    /// - common properties used in all of the implemented lidars.
    /// - methods providing data to create the lidar in RGL with default implementation (may be overridden for specific lidars)
    /// </summary>
    [Serializable]
    public abstract class BaseLidarConfiguration
    {
        [Tooltip("Geometry description of lidar array")]
        public LaserArray laserArray;

        [Tooltip("The horizontal resolution of laser array firings (in degrees)")]
        [Min(0)] public float horizontalResolution;

        [Tooltip("Min horizontal angle (left)")]
        [Range(-360.0f, 360.0f)] public float minHAngle;

        [Tooltip("Max horizontal angle (right)")]
        [Range(-360.0f, 360.0f)] public float maxHAngle;

        [Tooltip("Time between two consecutive firings of the whole laser array (in milliseconds). Usually, it consists of firing time for all the lasers and recharge time.")]
        [Min(0)] public float laserArrayCycleTime;

        [Tooltip("Represents the deviation of photons from a single beam emitted by a LiDAR sensor (in degrees). Used for simulating snow only (private feature).")]
        [Range(0.0f, 360.0f)] public float beamDivergence;

        [Tooltip("Lidar noise parameters")]
        public LidarNoiseParams noiseParams;

        public int HorizontalSteps => Math.Max((int)Math.Round(((maxHAngle - minHAngle) / horizontalResolution)), 1);
        public int PointCloudSize => laserArray.lasers.Length * HorizontalSteps;

        /// <summary>
        /// Returns ray poses for the single lidar scan.
        /// Poses are generated uniformly. May be overridden.
        /// </summary>
        public virtual Matrix4x4[] GetRayPoses()
        {
            Matrix4x4[] rayPoses = new Matrix4x4[PointCloudSize];
            Matrix4x4[] laserPoses = laserArray.GetLaserPoses();
            for (int hStep = 0; hStep < HorizontalSteps; hStep++)
            {
                for (int laserId = 0; laserId < laserPoses.Length; laserId++)
                {
                    int idx = laserId + hStep * laserPoses.Length;
                    float azimuth = minHAngle + hStep * horizontalResolution;
                    rayPoses[idx] = Matrix4x4.Rotate(Quaternion.Euler(0.0f, azimuth, 0.0f)) * laserPoses[laserId];
                }
            }
            return rayPoses;
        }

        /// <summary>
        /// Returns ranges for the generated rays.
        /// Ranges are retrieved from lasers description. May be overridden.
        /// </summary>
        public virtual Vector2[] GetRayRanges()
        {
            Vector2[] rayRanges = new Vector2[PointCloudSize];
            Vector2[] laserRanges = laserArray.GetLaserRanges();
            for (int i = 0; i < HorizontalSteps; i++)
            {
                Array.Copy(laserRanges, 0, rayRanges, i * laserRanges.Length, laserRanges.Length);
            }
            return rayRanges;
        }

        /// <summary>
        /// Returns time offsets for the generated rays.
        /// Time offsets are retrieved from lasers description. May be overridden.
        /// </summary>
        public virtual float[] GetRayTimeOffsets()
        {
            float[] rayTimeOffsets = new float[PointCloudSize];
            for (int hStep = 0; hStep < HorizontalSteps; hStep++)
            {
                for (int laserId = 0; laserId < laserArray.lasers.Length; laserId++)
                {
                    int idx = laserId + hStep * laserArray.lasers.Length;
                    rayTimeOffsets[idx] = laserArray.lasers[laserId].timeOffset + laserArrayCycleTime * hStep;
                }
            }
            return rayTimeOffsets;
        }

        /// <summary>
        /// Returns ring Ids for the generated rays.
        /// Ring Ids are retrieved from lasers description. May be overridden.
        /// </summary>
        public virtual int[] GetRayRingIds()
        {
            return laserArray.GetLaserRingIds();
        }

        /// <summary>
        /// Returns transform from the attached game object to the LiDAR origin.
        /// </summary>
        public Matrix4x4 GetLidarOriginTransfrom()
        {
            return Matrix4x4.Translate(laserArray.centerOfMeasurementLinearOffsetMm / 1000.0f);
        }
    }

    /// <summary>
    /// Lidar configuration for uniformly distributed rays along the horizontal axis with ranges retrieved from lasers description.
    /// It allows the definition of the lidar with different ranges for each laser (channel).
    /// </summary>
    [Serializable]
    public class LaserBasedRangeLidarConfiguration : BaseLidarConfiguration { }

    /// <summary>
    /// Lidar configuration for uniformly distributed rays along the horizontal axis with a uniform range for all the rays.
    /// Configuration introduces new properties for setting the minimum and maximum range of the lidar.
    /// </summary>
    [Serializable]
    public class UniformRangeLidarConfiguration : BaseLidarConfiguration
    {
        [Tooltip("Minimum range of the sensor")]
        [Min(0)] public float minRange;

        [Tooltip("Maximum range of the sensor")]
        [Min(0)] public float maxRange;

        public override Vector2[] GetRayRanges()
        {
            return new Vector2[1] {new Vector2(minRange, maxRange)};
        }
    }

    /// <summary>
    /// Lidar configuration for HesaiAT128 lidar.
    /// It contains properties and ray-generating methods specific to this lidar.
    /// </summary>
    [Serializable]
    public class HesaiAT128LidarConfiguration : BaseLidarConfiguration
    {
        public override Vector2[] GetRayRanges()
        {
            // All channels fire laser pulses that measure the far field (＞ 7.2 m)
            // Additionally, the NF-enabled channels also fire laser pulses that measure only the near field (0.5 to 7.2 m), at a time other
            // than these channels' far field firings.
            // NF-enabled channels are marked with minRange set to NEAR_FIELD_MIN_RANGE.
            const float NEAR_FIELD_MIN_RANGE = 0.5f;
            const float FAR_FIELD_MIN_RANGE = 7.2f;

            Vector2[] rayRanges = new Vector2[PointCloudSize];
            Vector2[] laserRanges = laserArray.GetLaserRanges();
            for (int i = 0; i < PointCloudSize; i++)
            {
                rayRanges[i] = laserRanges[i % laserRanges.Length];
                // The horizontal resolution for the near field is 2 times greater than for the far field.
                // To simulate it, every second minimum range of the ray will be set for the far field.
                if (rayRanges[i].x == NEAR_FIELD_MIN_RANGE && (i / laserRanges.Length) % 2 == 1)
                {
                    rayRanges[i].x = FAR_FIELD_MIN_RANGE;
                }
            }
            return rayRanges;
        }
    }

    /// <summary>
    /// Lidar configuration for HesaiQT128C2X lidar.
    /// It contains properties and ray-generating methods specific to this lidar.
    /// </summary>
    [Serializable]
    public class HesaiQT128C2XLidarConfiguration : BaseLidarConfiguration
    {
        private static int hesaiQT128LasersBankLength = 32;

        // Lasers (channels) in HesaiQT128C2X are divided into 4 banks: Bank A, Bank B, Bank C, Bank D.
        // Firing sequence for the one scan is:
        //   1. Banks CDB fire; Bank A rests
        //   2. The horizontal step moves by half of the horizontal resolution
        //   3. Banks CDA fire, Bank B rests
        // It results in high resolution (doubled resolution) for lasers in banks C and D.
        public override Matrix4x4[] GetRayPoses()
        {
            Matrix4x4[] rayPoses = new Matrix4x4[PointCloudSize];
            Matrix4x4[] laserPoses = laserArray.GetLaserPoses();
            for (int hStep = 0; hStep < HorizontalSteps; hStep++)
            {
                for (int laserId = 0; laserId < laserPoses.Length; laserId++)
                {
                    int idx = laserId + hStep * laserPoses.Length;
                    float highResolutionAddition = 0.0f;
                    if (laserId >= 3 * hesaiQT128LasersBankLength)
                    {
                        highResolutionAddition = horizontalResolution / 2;
                    }
                    float azimuth = minHAngle + hStep * horizontalResolution + highResolutionAddition;
                    rayPoses[idx] = Matrix4x4.Rotate(Quaternion.Euler(0.0f, azimuth, 0.0f)) * laserPoses[laserId];
                }
            }
            return rayPoses;
        }
    }

    /// <summary>
    /// Lidar configuration for HesaiPandar128E4X lidar.
    /// It contains properties and ray-generating methods specific to this lidar.
    /// </summary>
    [Serializable]
    public class HesaiPandar128E4XLidarConfiguration : BaseLidarConfiguration
    {
        private int nextSubstepLaserIdForHighRes = 95;

        private static readonly Dictionary<bool, LaserArray> HighResolutionModeToLaserArrayMapping =
            new Dictionary<bool, LaserArray>()
            {
                { false, LaserArrayLibrary.HesaiPandar128E4X },
                { true, LaserArrayLibrary.HesaiPandar128E4XHighRes }
            };

        public bool highResolutionMode;

        // Properties with custom setter cannot be serialized
        // This is a workaround to switch lasers if high resolution mode has changed
        // This method is called at the beginning of every ray-generating methods
        private void EnsureProperLasersAssigned()
        {
            if (laserArray.lasers.Length != HighResolutionModeToLaserArrayMapping[highResolutionMode].lasers.Length)
            {
                laserArray.lasers = HighResolutionModeToLaserArrayMapping[highResolutionMode].lasers;
            }
        }

        // In standard mode, rays are generated uniformly.
        // In high resolution mode, first rays (up to `nextSubstepLaserIdForHighRes`) are generated on standard horizontal angle
        // Next rays (after `nextSubstepLaserIdForHighRes`) are shifted by half of the horizontal resolution
        // Some lasers fire on both horizontal states. This is taken into account in the order of the lasers in `laserArray.lasers`.
        public override Matrix4x4[] GetRayPoses()
        {
            EnsureProperLasersAssigned();
            if (!highResolutionMode)
            {
                return base.GetRayPoses();
            }

            Matrix4x4[] rayPoses = new Matrix4x4[PointCloudSize];
            Matrix4x4[] laserPoses = laserArray.GetLaserPoses();
            for (int hStep = 0; hStep < HorizontalSteps; hStep++)
            {
                for (int laserId = 0; laserId < laserPoses.Length; laserId++)
                {
                    int idx = laserId + hStep * laserPoses.Length;
                    float highResolutionAddition = 0.0f;
                    if (laserId >= nextSubstepLaserIdForHighRes)
                    {
                        highResolutionAddition = horizontalResolution / 2;
                    }
                    float azimuth = minHAngle + hStep * horizontalResolution + highResolutionAddition;
                    rayPoses[idx] = Matrix4x4.Rotate(Quaternion.Euler(0.0f, azimuth, 0.0f)) * laserPoses[laserId];
                }
            }
            return rayPoses;
        }

        public override Vector2[] GetRayRanges()
        {
            EnsureProperLasersAssigned();
            return base.GetRayRanges();
        }

        public override float[] GetRayTimeOffsets()
        {
            EnsureProperLasersAssigned();
            return base.GetRayTimeOffsets();
        }

        public override int[] GetRayRingIds()
        {
            EnsureProperLasersAssigned();
            return base.GetRayRingIds();
        }
    }
}
