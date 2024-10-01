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
    /// <summary>
    /// The singleton component to activate and configure the snow model for the LiDAR sensors.
    /// Note: This is a private feature. It requires replacing the RGL binary with the RGL private build binary.
    /// </summary>
    public class LidarSnowManager : MonoBehaviour
    {
        // Singleton pattern
        public static LidarSnowManager Instance { get; private set; }

        // Delegate to notify that the snow model has been changed
        public delegate void OnNewConfigDelegate();
        public OnNewConfigDelegate OnNewConfig;

        [field: Header("Base Settings")]

        [field: SerializeField]
        public bool IsSnowEnabled { get; private set; } = false;

        // Snow model properties
        [field: SerializeField]
        [field: Tooltip("The precipitation rate for snow is expressed in rate of equivalent water depth in mm per hour")]
        [field: Range(0.0f, 50.0f)]
        public float RainRate { get; private set; } = 9.0f;

        [field: SerializeField]
        [field: Tooltip("Mean diameter of snowflakes in meters")]
        [field: Range(0.0001f, 0.01f)]
        public float MeanSnowflakeDiameter { get; private set; } = 0.003f;

        [field: SerializeField]
        [field: Tooltip("Terminal velocity of snowflakes in meters per second")]
        [field: Range(0.2f, 2.0f)]
        public float TerminalVelocity { get; private set; } = 1.6f;

        [field: SerializeField]
        [field: Tooltip("Density of snowflakes in g per cubic centimeter depending on snow wetness")]
        [field: Range(0.01f, 0.2f)]
        public float Density { get; private set; } = 0.07f;

        [field: SerializeField]
        [field: Tooltip("Minimal beam aperture occupancy (ratio) that means a hit, both for snowflakes and for original hit")]
        [field: Range(0.0f, 1.0f)]
        public float OccupancyThreshold { get; private set; } = 0.0f;

        [field: Header("Defaults")]

        [field: SerializeField]
        [field: Tooltip("Entity ID that is assigned to cloud points resulting from snowflake hits")]
        public int SnowflakesId { get; private set; } = 268435455; // Default RGL entity ID.

        [field: SerializeField]
        [field: Tooltip("Initial intensity of each LiDAR laser beam, used to evaluate energy loss based on beam aperture occupancy")]
        [field: Min(0.0f)]
        public float FullBeamIntensity { get; private set; } = 1.0f;

        private void Awake()
        {
            if (!IsSnowFeatureAvailable())
            {
                Debug.LogError("Loaded RGL plugin does not include support for Lidar Snow Model, removing component");
                Destroy(this);
                return;
            }

            if (Instance != null && Instance != this)
            {
                Debug.LogError("LidarSnowManager is already on the scene. Removing this component");
                Destroy(this);
                return;
            }
            Instance = this;
        }

        private void OnValidate()
        {
            OnNewConfig?.Invoke();
        }

        private void OnEnable()
        {
            IsSnowEnabled = true;
            OnNewConfig?.Invoke();
        }

        private void OnDisable()
        {
            IsSnowEnabled = false;
            OnNewConfig?.Invoke();
        }

        public bool IsSnowFeatureAvailable()
        {
            return RGLNativeAPI.HasExtension(RGLExtension.RGL_EXTENSION_WEATHER);
        }
    }
}
