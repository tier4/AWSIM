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

using UnityEngine;

namespace RGLUnityPlugin
{
    [System.Serializable]
    public struct RadarNoiseParams
    {
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
    }
}