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
using System.Linq;
using UnityEngine;

namespace RGLUnityPlugin
{
    [Serializable]
    public struct RadarScopeParameters
    {
        [Tooltip("Beginning of distance range for this scope")]
        [Min(0.0f)] public float beginDistance;
        [Tooltip("End of distance range for this scope")]
        [Min(0.0f)] public float endDistance;
        [Tooltip("Minimum distance difference to create separate detections")]
        [Min(0.0f)] public float distanceSeparationThreshold;
        [Tooltip("Minimum radial speed difference to create separate detections")]
        [Min(0.0f)] public float radialSpeedSeparationThreshold;
        [Tooltip("Minimum azimuth difference to create separate detections")]
        [Min(0.0f)] public float azimuthSeparationThreshold;
    }

    [Serializable]
    public class RadarConfiguration
    {
        [Tooltip("Minimum azimuth angle (left)")]
        [Range(-180.0f, 180.0f)] public float minAzimuthAngle;

        [Tooltip("Maximum azimuth angle (right)")]
        [Range(-180.0f, 180.0f)] public float maxAzimuthAngle;

        [Tooltip("Minimum elevation angle (down)")]
        [Range(-180.0f, 180.0f)] public float minElevationAngle;

        [Tooltip("Maximum elevation angle (up)")]
        [Range(-180.0f, 180.0f)] public float maxElevationAngle;

        [Tooltip("List of radar scope parameters")]
        public RadarScopeParameters[] scopeParametersList;

        [Tooltip("Radar noise parameters")]
        public RadarNoiseParams noiseParams;

        [NonSerialized]
        public float azimuthResolution = 0.49f;
        [NonSerialized]
        public float elevationResolution = 0.49f;

        private int azimuthStepCount => Math.Max((int)Math.Round((maxAzimuthAngle - minAzimuthAngle) / azimuthResolution), 1);
        private int elevationStepCount => Math.Max((int)Math.Round((maxElevationAngle - minElevationAngle) / elevationResolution), 1);
        private int pointCloudSize => elevationStepCount * azimuthStepCount;

        public Matrix4x4[] GetRayPoses()
        {
            if (!(minAzimuthAngle <= maxAzimuthAngle))
            {
                throw new ArgumentOutOfRangeException(nameof(minAzimuthAngle),
                    "Minimum angle must be lower or equal to the maximum angle");
            }

            if (!(minElevationAngle <= maxElevationAngle))
            {
                throw new ArgumentOutOfRangeException(nameof(minElevationAngle),
                    "Minimum angle must be lower or equal to the maximum angle");
            }

            Matrix4x4[] rayPose = new Matrix4x4[pointCloudSize];
            for (int aStep = 0; aStep < azimuthStepCount; aStep++)
            {
                for (int eStep = 0; eStep < elevationStepCount; eStep++)
                {
                    int idx = eStep + aStep * elevationStepCount;
                    float azimuth = Mathf.Min(minAzimuthAngle + aStep * azimuthResolution, maxAzimuthAngle);
                    float elevation = Mathf.Min(minElevationAngle + eStep * elevationResolution, maxElevationAngle);
                    rayPose[idx] = Matrix4x4.Rotate(Quaternion.Euler(elevation, azimuth, 0.0f));
                }
            }
            return rayPose;
        }

        public Vector2[] GetRayRanges()
        {
            float minRange = scopeParametersList.Min(s => s.beginDistance);
            float maxRange = scopeParametersList.Max(s => s.endDistance);

            return new[] { new Vector2(minRange, maxRange) };
        }
    }
}
