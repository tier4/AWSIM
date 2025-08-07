// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using Awsim.Common;

namespace Awsim.Entity
{
    /// <summary>
    /// Traffic Light class.
    /// </summary>
    public class TrafficLight : MonoBehaviour
    {
        /// <summary>
        /// Type of each bulb.
        /// </summary>
        public enum BulbType
        {
            AnyCircleBulb = 0,
            RedBulb = 1,
            YellowBulb = 2,
            GreenBulb = 3,
            LeftArrowBulb = 4,
            RightArrowBulb = 5,
            UpArrowBulb = 6,
            DownArrowBulb = 7,
            DownLeftArrowBulb = 8,
            DownRightArrowBulb = 9,
            CrossBulb = 10,
        }

        /// <summary>
        /// Bulb lighting status.
        /// </summary>
        public enum BulbStatus
        {
            SolidOff = 0,        // Lights off.
            SolidOn = 1,        // Lights on.
            Frashing = 2,        // Lights on every flashSec.
        }

        /// <summary>
        /// Bulb lighting color.
        /// </summary>
        public enum BulbColor
        {
            Red = 0,
            Yellow = 1,
            Green = 2,
            White = 3,
        }

        /// <summary>
        /// Used in TrafficLight.SetBulbData(). Based on the data in this class, the lighting of each bulb is controlled.
        /// </summary>
        [Serializable]
        public struct BulbData
        {
            public BulbType Type => _type;

            public BulbColor Color => _color;

            public BulbStatus Status => _status;

            [SerializeField] BulbType _type;

            [SerializeField] BulbColor _color;

            [SerializeField] BulbStatus _status;

            public BulbData(BulbType type, BulbColor color, BulbStatus status)
            {
                this._type = type;
                this._color = color;
                this._status = status;
            }
        }

        /// <summary>
        /// Define TrafficLight bulbs.
        /// </summary>
        [Serializable]
        class Bulb
        {
            /// <summary>
            /// Emission configuration to be applied to the material when the bulb is lit.
            /// </summary>
            [Serializable]
            public class EmissionConfig
            {
                public BulbColor BulbColor;
                public Color Color;
                public float Intensity;
                [Range(0, 1)] public float ExposureWeight;
            }

            public BulbType BulbType => _bulbType;
            public BulbStatus BulbStatus => _status;
            public BulbColor BulbColor => _color;

            [SerializeField]
            BulbType _bulbType;

            [SerializeField, Tooltip("Specifies the index of the material to be used for the bulb.")]
            int _materialIndex;

            // const parameters.
#if HDRP
            const string _emissiveColor = "_EmissiveColor";
            const string _emissiveIntensity = "_EmissiveIntensity";
            const string _emissiveExposureWeight = "_EmissiveExposureWeight";
#elif URP
            const string _emissiveColor = "_EmissionColor";
#endif

            const string _lightOnFlag = "_LightOn";
            const float _flashIntervalSec = 0.5f;                // flash bulb lighting interval(sec).

            float _timer = 0;                            // used for flashing status.     NOTE: Might as well make it static and refer to the same time. 
            Color _defaultEmissiveColor;                 // default bulb material emissive color.
            float _defaultEmissiveExposureWeight;        // default bulb mateiral emissive exposure weight
            Dictionary<BulbColor, EmissionConfig> _bulbColorConfigPairs;
            Material _material = null;                   // bulb mateiral(instance).
            bool _initialized = false;
            BulbStatus _status = BulbStatus.SolidOff;
            BulbColor _color;
            bool _isLightOn = false;                     // used for flashing control.

            /// <summary>
            /// Called from TrafficLight class. Acquire and initialize bulb material.
            /// </summary>
            /// <param name="renderer">Renderer containing the bulb material.</param>
            /// <param name="bulbEmissionConfigs"></param>

            public void Initialize(Renderer renderer, EmissionConfig[] bulbEmissionConfigs)
            {
                // bulb color config.
                _bulbColorConfigPairs = bulbEmissionConfigs.ToDictionary(x => x.BulbColor);

                // set material.
                _material = renderer.materials[_materialIndex];

#if HDRP
                _defaultEmissiveColor = _material.GetColor(_emissiveColor);
                _defaultEmissiveExposureWeight = _material.GetFloat(_emissiveExposureWeight);
#elif URP
                _defaultEmissiveColor = _material.GetColor(_emissiveColor);
                
#endif
                _initialized = true;
            }

            /// <summary>
            /// Called from TrafficLight class.
            /// Set the bulb lighting.
            /// </summary>
            /// <param name="status">bulb status</param>
            /// <param name="color">bulb color</param>
            public void SetBulbLighting(BulbStatus status, BulbColor color)
            {
                if (_initialized == false)
                {
                    Debug.LogError("Not initialized Traffic bulb.");
                    return;
                }

                if (this._status == status && this._color == color)
                    return;

                this._status = status;
                this._color = color;

                switch (status)
                {
                    case BulbStatus.SolidOn:
                        _timer = 0;
                        Set(true); break;
                    case BulbStatus.SolidOff:
                        _timer = 0;
                        Set(false); break;
                    case BulbStatus.Frashing:
                        _timer = 0;
                        Set(true); break;
                }
            }

            /// <summary>
            /// Called from TrafficLight class. 
            /// Update timer for bulb flashing.
            /// </summary>
            /// <param name="deltaTime"></param>
            public void Update(float deltaTime)
            {
                if (_status != BulbStatus.Frashing)
                    return;

                _timer += deltaTime;

                if (_timer > _flashIntervalSec)
                {
                    Set(!_isLightOn);
                }
            }

            /// <summary>
            ///  Called from TrafficLight class. 
            ///  Discard the material instance.
            /// </summary>
            public void DestroyMaterial()
            {
                UnityObject.DestroySafe(ref _material);
                _initialized = false;
            }

            /// <summary>
            /// Change material parameters to control lighting.
            /// </summary>
            /// <param name="isLightOn">Light up bulb</param>
            void Set(bool isLightOn)
            {
                var colorToSet = isLightOn
                    ? _bulbColorConfigPairs[_color].Color * _bulbColorConfigPairs[_color].Intensity
                    : _defaultEmissiveColor;

#if HDRP
                _material.SetColor(_emissiveColor, colorToSet);
                var exposureWeight = isLightOn
                    ? _bulbColorConfigPairs[_color].ExposureWeight
                    : _defaultEmissiveExposureWeight;

                _material.SetFloat(_emissiveExposureWeight, exposureWeight);
#elif URP
                _material.SetColor(_emissiveColor, colorToSet);
                if (isLightOn)
                {
                    _material.EnableKeyword("_EMISSION");
                }
                else
                {
                    _material.DisableKeyword("_EMISSION");
                }
#endif

                if (_material.HasProperty(_lightOnFlag))
                {
                    _material.SetInt(_lightOnFlag, isLightOn ? 1 : 0);
                }

                _isLightOn = isLightOn;
                _timer = 0;
            }

        }

        [SerializeField, Tooltip("Set the Renderer containing the bulb material.")]
        Renderer _renderer;

        /// <summary>
        /// Define the Emission parameter to be applied to the material when the Bulb is turned on.
        /// </summary>
        [Header("Bulb Emission config")]
        [SerializeField, Tooltip("Define the Emission parameter for BulbColor.")]
        Bulb.EmissionConfig[] _bulbEmissionConfigs = new Bulb.EmissionConfig[]
        {
            new Bulb.EmissionConfig()
            {
                BulbColor = BulbColor.Green,
                Color = Color.green,
                Intensity = 14,
                ExposureWeight = 0.8f,
            },
            new Bulb.EmissionConfig()
            {
                BulbColor = BulbColor.Yellow,
                Color = Color.yellow,
                Intensity = 14,
                ExposureWeight = 0.8f,
            },
            new Bulb.EmissionConfig()
            {
                BulbColor = BulbColor.Red,
                Color = Color.red,
                Intensity = 14,
                ExposureWeight = 0.8f,
            },
            new Bulb.EmissionConfig()
            {
                BulbColor = BulbColor.White,
                Color = Color.white,
                Intensity = 14,
                ExposureWeight = 0.8f,
            },
        };

        [Header("Bulb material config"), Tooltip("Link the material of the bulb to the type.")]
        [SerializeField] Bulb[] _bulbs;

        Dictionary<BulbType, Bulb> _bulbPairs;
        int _bulbCount;
        BulbData[] _bulbDataArray;

        void Reset()
        {
            _renderer = GetComponent<Renderer>();
        }

        public void Initialize()
        {
            _bulbPairs = _bulbs.ToDictionary(x => x.BulbType);
            _bulbCount = _bulbPairs.Count();
            _bulbDataArray = new BulbData[_bulbCount];

            // Initialize bulb materials.
            foreach (var e in _bulbPairs.Values)
            {
                e.Initialize(_renderer, _bulbEmissionConfigs);
            }
        }

        public void OnUpdate()
        {
            // Update timers for each Bulb.
            foreach (var e in _bulbPairs.Values)
            {
                e.Update(Time.deltaTime);
            }
        }

        public void TurnOffAllBulbs()
        {
            foreach (var e in _bulbs)
            {
                e.SetBulbLighting(TrafficLight.BulbStatus.SolidOff, BulbColor.White);
            }
        }

        /// <summary>
        /// Updates the status of each bulb of this traffic light.
        /// </summary>
        /// <param name="inputDatas">Input data to update each bulb.</param>
        public void SetBulbData(BulbData[] inputDatas)
        {
            for (int i = 0; i < inputDatas.Length; i++)
            {
                var inputData = inputDatas[i];
                SetBulbData(inputData);
            }
        }

        /// <summary>
        /// Updates the status of each bulb of this traffic light.
        /// </summary>
        /// <param name="inputData">Input data to update each bulb.</param>
        public void SetBulbData(BulbData inputData)
        {
            _bulbPairs[inputData.Type].SetBulbLighting(inputData.Status, inputData.Color);
        }

        /// <summary>
        /// Get the current status of each bulb of the traffic light.
        /// </summary>
        /// <returns>bulb data array</returns>
        public BulbData[] GetBulbData()
        {
            int i = 0;

            foreach (var e in _bulbPairs)
            {
                _bulbDataArray[i] = new BulbData(e.Value.BulbType, e.Value.BulbColor, e.Value.BulbStatus);
                i++;
            }

            return _bulbDataArray;
        }

        void OnDestroy()
        {
            // Destory bulb materials.
            foreach (var e in _bulbs)
            {
                e.DestroyMaterial();
            }
        }
    }
}