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
using UnityEngine;


namespace RGLUnityPlugin
{
    [Serializable]
    public struct LidarConfiguration
    {
        /// <summary>
        /// Geometry description of lidar array
        /// </summary>
        public LaserArray laserArray;

        /// <summary>
        /// The number of laser array firings between minHAngle and maxHAngle
        /// </summary>
        [Min(1)] public int horizontalSteps;

        /// <summary>
        /// Min horizontal angle (left)
        /// </summary>
        public float minHAngle;

        /// <summary>
        /// Max horizontal angle (right)
        /// </summary>
        public float maxHAngle;

        /// <summary>
        /// Maximum range of the sensor.
        /// </summary>
        [Min(0)] public float maxRange;

        /// <summary>
        /// Lidar noise paramteres
        /// </summary>
        public LidarNoiseParams noiseParams;

        public int PointCloudSize => laserArray.lasers.Length * horizontalSteps;

        public Matrix4x4[] GetRayPoses()
        {
            if (!(minHAngle <= maxHAngle))
            {
                throw new ArgumentOutOfRangeException(nameof(minHAngle),
                    "Minimum angle must be lower or equal to maximum angle");
            }

            if (!(horizontalSteps > 0))
            {
                throw new ArgumentOutOfRangeException(nameof(horizontalSteps),
                    "Number of horizontal steps must be positive");
            }

            Matrix4x4[] rayPose = new Matrix4x4[PointCloudSize];
            float azimuthDelta = (maxHAngle - minHAngle) / horizontalSteps;

            Matrix4x4[] laserPoses = laserArray.GetLaserPoses();
            for (int hStep = 0; hStep < horizontalSteps; hStep++)
            {
                for (int laserId = 0; laserId < laserPoses.Length; laserId++)
                {
                    int idx = laserId + hStep * laserPoses.Length;
                    float azimuth = minHAngle + hStep * azimuthDelta;
                    rayPose[idx] = Matrix4x4.Rotate(Quaternion.Euler(0.0f, azimuth, 0.0f)) * laserPoses[laserId];
                }
            }

            return rayPose;
        }

        public static LidarNoiseParams TypicalNoiseParams => new LidarNoiseParams
        {
            angularNoiseType = AngularNoiseType.RayBased,
            angularNoiseMean = Mathf.Rad2Deg * 0.0f,
            angularNoiseStDev = Mathf.Rad2Deg * 0.001f,
            distanceNoiseStDevBase = 0.02f,
            distanceNoiseStDevRisePerMeter = 0.0f,
            distanceNoiseMean = 0.0f,
        };

        public static LidarNoiseParams ZeroNoiseParams => new LidarNoiseParams
        {
            angularNoiseType = AngularNoiseType.RayBased,
            angularNoiseStDev = 0.0f,
            distanceNoiseStDevBase = 0.0f,
            distanceNoiseStDevRisePerMeter = 0.0f,
            angularNoiseMean = 0.0f,
            distanceNoiseMean = 0.0f,
        };
    }
}