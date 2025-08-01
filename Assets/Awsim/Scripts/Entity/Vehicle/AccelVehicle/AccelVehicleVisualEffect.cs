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
using UnityEngine;
using Awsim.Common;

namespace Awsim.Entity
{
    public class AccelVehicleVisualEffect : MonoBehaviour
    {
        [Serializable]
        public class EmissionMaterial
        {
            [SerializeField] MeshRenderer _meshRenderer;
            [SerializeField] int _materialIndex;
            [SerializeField] float _lightingIntensity;
            [SerializeField] Color _lightingColor;
            [SerializeField, Range(0, 1)] float _lightingExposureWeight;

            Material _material = null;
            Color _defaultEmissiveColor;
            float _defaultExposureWeight;
            bool _isOn = false;

#if HDRP
            const string _emissiveColor = "_EmissiveColor";
            const string _emissiveExposureWeight = "_EmissiveExposureWeight";

#elif URP
            const string _emissiveColor = "_EmissionColor";
#endif
            public void Initialize()
            {
                if (_material != null)
                    return;

                _material = _meshRenderer.materials[_materialIndex];
                _material.EnableKeyword("_EMISSION");

#if HDRP
                _defaultEmissiveColor = _material.GetColor(_emissiveColor);
                _defaultExposureWeight = _material.GetFloat(_emissiveExposureWeight);
#elif URP
                _defaultEmissiveColor = _material.GetColor(_emissiveColor);
#endif
            }

            public void Set(bool isLightOn)
            {
                if (_isOn == isLightOn)
                    return;

                _isOn = isLightOn;

                var targetColor = isLightOn ? _lightingColor * _lightingIntensity : _defaultEmissiveColor;

#if HDRP
                _material.SetColor(_emissiveColor, targetColor);
                var exposureWeight = isLightOn ? _lightingExposureWeight : _defaultExposureWeight;
                _material.SetFloat(_emissiveExposureWeight, exposureWeight);
#elif URP
                _material.SetColor(_emissiveColor, targetColor);
#endif
            }

            public void DestroyMaterial()
            {
                if (_material != null)
                    UnityObject.DestroySafe(ref _material);
            }
        }

        [Header("Vehicle")]
        [SerializeField] AccelVehicle _vehicle;

        [Header("Brake Light")]
        [SerializeField] EmissionMaterial[] _brakeLights;

        [Header("Turn Signal")]
        [SerializeField] EmissionMaterial[] _leftTurnSignalLights;
        [SerializeField] EmissionMaterial[] _rightTurnSignalLights;
        [SerializeField] float _turnSingalTimerIntervalSec = 0.5f;
        float _turnSignalTimer = 0f;
        bool _isTurnSignalOnInterval = false;


        [Header("Reverse Light")]
        [SerializeField] EmissionMaterial[] _reverseLights;

        public void Initialize()
        {
            foreach (var e in _brakeLights) e.Initialize();
            foreach (var e in _leftTurnSignalLights) e.Initialize();
            foreach (var e in _rightTurnSignalLights) e.Initialize();
            foreach (var e in _reverseLights) e.Initialize();
        }

        public void OnUpdate()
        {
            // TODO: Implement steering object rotation.

            // Brake light.
            ApplyLights(_brakeLights, IsBrakeLight());

            // Reverse light.
            ApplyLights(_reverseLights, IsReverseLight());

            // Turn indicator & hazard light.
            if (!IsTurnSignalOn())
            {
                if (_turnSignalTimer != 0)
                    _turnSignalTimer = 0;

                if (_isTurnSignalOnInterval != false)
                    _isTurnSignalOnInterval = false;

                ApplyLights(_leftTurnSignalLights, false);
                ApplyLights(_rightTurnSignalLights, false);

                return;
            }

            _turnSignalTimer -= Time.deltaTime;
            if (_turnSignalTimer < 0f)
            {
                _turnSignalTimer = _turnSingalTimerIntervalSec;
                _isTurnSignalOnInterval = !_isTurnSignalOnInterval;
            }

            ApplyLights(_leftTurnSignalLights, IsTurnLeftLgiht());
            ApplyLights(_rightTurnSignalLights, IsTurnRightLight());


            // --- Internal methods ---
            void ApplyLights(EmissionMaterial[] emissionMaterials, bool isOn)
            {
                foreach (var e in emissionMaterials)
                    e.Set(isOn);
            }

            bool IsBrakeLight()
            {
                return (_vehicle.Gear == Gear.Drive && _vehicle.AccelerationInput < 0) ||
                       (_vehicle.Gear == Gear.Reverse && _vehicle.AccelerationInput < 0);
            }

            bool IsReverseLight()
            {
                return _vehicle.Gear == Gear.Reverse;
            }

            bool IsTurnSignalOn()
            {
                return _vehicle.TurnIndicators == TurnIndicators.Left ||
                       _vehicle.TurnIndicators == TurnIndicators.Right ||
                       _vehicle.HazardLights == HazardLights.Enable;
            }

            bool IsTurnLeftLgiht()
            {
                return (_vehicle.TurnIndicators == TurnIndicators.Left || _vehicle.HazardLights == HazardLights.Enable) &&
                        _isTurnSignalOnInterval;
            }

            bool IsTurnRightLight()
            {
                return (_vehicle.TurnIndicators == TurnIndicators.Right || _vehicle.HazardLights == HazardLights.Enable) &&
                        _isTurnSignalOnInterval;
            }
        }
    }
}