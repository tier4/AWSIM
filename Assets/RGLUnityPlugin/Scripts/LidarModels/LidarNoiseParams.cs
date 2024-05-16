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
    public enum AngularNoiseType
    {
        RayBased = 0,
        HitpointBased = 1
    }

    [System.Serializable]
    public struct LidarNoiseParams : IEquatable<LidarNoiseParams>
    {
        [Tooltip("Angular noise type")]
        public AngularNoiseType angularNoiseType;

        [Min(0), Tooltip("Angular noise standard deviation in degrees")]
        public float angularNoiseStDev; // Degrees

        [Tooltip("Angular noise mean in degrees")]
        public float angularNoiseMean; // Degrees

        [Min(0), Tooltip("Distance noise standard deviation base in meters")]
        public float distanceNoiseStDevBase; // Meters

        [Min(0), Tooltip("Distance noise standard deviation rise per meter")]
        public float distanceNoiseStDevRisePerMeter; // Meters

        [Tooltip("Distance noise mean in meters")]
        public float distanceNoiseMean; // Meters

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

        //// IEquatable interface
        public bool Equals(LidarNoiseParams other)
        {
            return this.angularNoiseType == other.angularNoiseType &&
                   this.angularNoiseStDev == other.angularNoiseStDev &&
                   this.angularNoiseMean == other.angularNoiseMean &&
                   this.distanceNoiseStDevBase == other.distanceNoiseStDevBase &&
                   this.distanceNoiseStDevRisePerMeter == other.distanceNoiseStDevRisePerMeter &&
                   this.distanceNoiseMean == other.distanceNoiseMean;
        }

        public override bool Equals(object obj)
        {
            return obj is LidarNoiseParams equatable && Equals(equatable);
        }

        public override int GetHashCode() => (angularNoiseType, angularNoiseStDev, angularNoiseMean, distanceNoiseStDevBase, distanceNoiseStDevRisePerMeter, distanceNoiseMean).GetHashCode();
    }
}
