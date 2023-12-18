// Copyright 2023 Robotec.ai.
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
    public class RadarConfiguration
    {
        [Min(0.0f)] public float MinRange;

        [Min(0.0f)] public float MaxRange;

        [Range(-180.0f, 180.0f)] public float MinAzimuthAngle;

        [Range(-180.0f, 180.0f)] public float MaxAzimuthAngle;

        [Range(-180.0f, 180.0f)] public float MinElevationAngle;

        [Range(-180.0f, 180.0f)] public float MaxElevationAngle;

        public float RangeSeparation;
        public float AzimuthSeparation;

        public RadarNoiseParams NoiseParams;

        private float azimuthResolution = 0.49f;
        private float elevationResolution = 0.49f;

        private int azimuthSteps => Math.Max((int)Math.Round(((MaxAzimuthAngle - MinAzimuthAngle) / azimuthResolution)), 1);
        private int elevationSteps => Math.Max((int)Math.Round(((MaxElevationAngle - MinElevationAngle) / elevationResolution)), 1);
        private int pointCloudSize => elevationSteps * azimuthSteps;

        public Matrix4x4[] GetRayPoses()
        {
            if (!(MinAzimuthAngle <= MaxAzimuthAngle))
            {
                throw new ArgumentOutOfRangeException(nameof(MinAzimuthAngle),
                    "Minimum angle must be lower or equal to the maximum angle");
            }

            if (!(MinElevationAngle <= MaxElevationAngle))
            {
                throw new ArgumentOutOfRangeException(nameof(MinElevationAngle),
                    "Minimum angle must be lower or equal to the maximum angle");
            }

            Matrix4x4[] rayPose = new Matrix4x4[pointCloudSize];
            for (int aStep = 0; aStep < azimuthSteps; aStep++)
            {
                for (int eStep = 0; eStep < elevationSteps; eStep++)
                {
                    int idx = eStep + aStep * elevationSteps;
                    float azimuth = MinAzimuthAngle + aStep * azimuthResolution;
                    float elevation = MinElevationAngle + eStep * elevationResolution;
                    rayPose[idx] = Matrix4x4.Rotate(Quaternion.Euler(elevation, azimuth, 0.0f));
                }
            }
            return rayPose;
        }

        public Vector2[] GetRayRanges()
        {
            if (!(MinRange <= MaxRange))
            {
                throw new ArgumentOutOfRangeException(nameof(MinRange),
                    "Minimum range must be lower or equal to the maximum range");
            }

            return new Vector2[1] { new Vector2(MinRange, MaxRange) };
        }

        public static RadarNoiseParams TypicalNoiseParams => new RadarNoiseParams
        {
            angularNoiseMean = Mathf.Rad2Deg * 0.0f,
            angularNoiseStDev = Mathf.Rad2Deg * 0.001f,
            distanceNoiseStDevBase = 0.02f,
            distanceNoiseStDevRisePerMeter = 0.0f,
            distanceNoiseMean = 0.0f,
        };
    }
}
