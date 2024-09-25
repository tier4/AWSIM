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
    /// The singleton component to activate and configure the fog model for the LiDAR sensors.
    /// Note: This is a private feature. It requires replacing the RGL binary with the RGL private build binary.
    /// </summary>
    public class LidarFogManager : MonoBehaviour
    {
        // Singleton pattern
        public static LidarFogManager Instance { get; private set; }

        // Delegate to notify that the snow model has been changed
        public delegate void OnNewConfigDelegate();
        public OnNewConfigDelegate OnNewConfig;

        [field: Header("Base Settings")]

        [field: SerializeField]
        [field: Tooltip("Enable/disable fog effect on devices.r")]
        public bool IsFogEnabled { get;  set; } = false;

        [field: SerializeField]
        [field: Tooltip("The attenuation coefficient for fog corresponding to amount of fog in the air")]
        [field: Range(0.003f, 0.5f)]
        public float AttenuationCoefficient { get; private set; } = 0.03f;

        [field: SerializeField]
        [field: Tooltip("Near cross-point of transmitter and receiver beams of Lidar device in meters. Model assumes that lidar is working in bistatic beam configuration.")]
        [field: Range(0.0f, 1.0f)]
        public float NearCrossPoint { get; private set; } = 0.01f;

        [field: SerializeField]
        [field: Tooltip("Far cross-point of transmitter and receiver beams of Lidar device. Model assumes that lidar is working in bistatic beam configuration.")]
        [field: Range(1.0f, 10.0f)]
        public float FarCrossPoint { get; private set; } = 2.0f;

        [field: Header("Defaults")]
        [field: SerializeField]
        [field: Tooltip("Entity ID that is assigned to cloud points resulting from fog hits")]
        public int FogId { get; private set; } = 268435455; // Default RGL entity ID.

        private void Awake()
        {
            if (!IsFogFeatureAvailable())
            {
                Debug.LogError("Loaded RGL plugin does not include support for Lidar Fog Model, removing component");
                Destroy(this);
                return;
            }

            if (Instance != null && Instance != this)
            {
                Debug.LogError("LidarFogManager is already on the scene. Removing this component");
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
            IsFogEnabled = true;
            OnNewConfig?.Invoke();
        }

        private void OnDisable()
        {
            IsFogEnabled = false;
            OnNewConfig?.Invoke();
        }

        public bool IsFogFeatureAvailable()
        {
            return RGLNativeAPI.HasExtension(RGLExtension.RGL_EXTENSION_WEATHER);
        }
    }
}