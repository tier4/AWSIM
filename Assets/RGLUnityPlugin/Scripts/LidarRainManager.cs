// Copyright 2024 Robotec.ai.
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
    /// The singleton component to activate and configure the rain model for the LiDAR sensors.
    /// Note: This is a private feature. It requires replacing the RGL binary with the RGL private build binary.
    /// </summary>
    public class LidarRainManager : MonoBehaviour
    {
        // Singleton pattern
        public static LidarRainManager Instance { get; private set; }

        // Delegate to notify that the snow model has been changed
        public delegate void OnNewConfigDelegate();
        public OnNewConfigDelegate OnNewConfig;

        [field: SerializeField]
        [field: Tooltip("Enable/disable rain effect on devices.r")]
        public bool IsRainEnabled { get;  set; } = false;

        // Snow model properties
        [field: SerializeField]
        [field: Tooltip("The precipitation rate for rain is expressed in mm per hour")]
        [field: Range(0.0f, 50.0f)]
        public float RainRate { get; private set; } = 9.0f;

        [field: SerializeField]
        [field: Tooltip("If true, a more sophisticated method is used, which takes into account the energy loss of the lidar beam when hitting snowflakes")]
        public bool DoSimulateEnergyLoss { get; private set; } = true;

        [field: SerializeField]
        [field: Tooltip("Maximal number of droplets which not obstruct the lidar beam")]
        [field: Range(1, 10)]
        public int RainNumericalThreshold { get; private set; } = 1;

        private void Awake()
        {
            if (!IsRainFeatureAvailable())
            {
                Debug.LogError("Loaded RGL plugin does not include support for Lidar Rain Model, removing component");
                Destroy(this);
                return;
            }

            if (Instance != null && Instance != this)
            {
                Debug.LogError("LidarRainManager is already on the scene. Removing this component");
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
            IsRainEnabled = true;
            OnNewConfig?.Invoke();
        }

        private void OnDisable()
        {
            IsRainEnabled = false;
            OnNewConfig?.Invoke();
        }

        public bool IsRainFeatureAvailable()
        {
            return RGLNativeAPI.HasExtension(RGLExtension.RGL_EXTENSION_WEATHER);
        }
    }
}
