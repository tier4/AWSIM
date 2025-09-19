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
using System.Collections.Generic;
using UnityEngine;
using Awsim.Common;

namespace Awsim.Entity
{
    public class V2I : MonoBehaviour
    {
        public class OutputData
        {
            public LaneletTrafficLight[] trafficLights;
        }

        public Action<OutputData> OnOutput { get; set; } = null;

        [SerializeField]
        [Tooltip("traffic_signals publication frequency - number of publications per second")]
        [Range(1.0f, 100.0f)]
        int _outputHz = 10;

        [SerializeField]
        Transform _egoVehicleTransform;

        [SerializeField]
        double _egoDistanceToTrafficSignals = 150.0;

        LaneletTrafficLight[] _allTrafficLights;

        OutputData _outputData = new OutputData();

        float _timer = 0;

        public void Initialize()
        {
            _outputData.trafficLights = new List<LaneletTrafficLight>().ToArray();
            _allTrafficLights = GameObject.FindObjectsByType<LaneletTrafficLight>(FindObjectsSortMode.InstanceID);
        }

        public void OnFixedUpdate()
        {
            // Update timer.
            _timer += Time.deltaTime;

            // Matching output to hz.
            var interval = 1.0f / (int)_outputHz;
            interval -= 0.00001f;
            if (_timer < interval)
                return;
            _timer = 0;

            if (_egoVehicleTransform != null)
            {
                _outputData.trafficLights = FindClosestTrafficLights(_allTrafficLights, _egoVehicleTransform.position,
                    _egoDistanceToTrafficSignals);
            }
            OnOutput?.Invoke(_outputData);

            LaneletTrafficLight[] FindClosestTrafficLights(LaneletTrafficLight[] trafficLights, Vector3 position, double radius = 10.0)
            {
                List<LaneletTrafficLight> filteredLights = new List<LaneletTrafficLight>();

                for (int i = 0; i < trafficLights.Length; i++)
                {
                    var distance2D = LaneletGeometryUtility.Distance2D(trafficLights[i].transform.position, position);

                    // TODO: Maybe it would be better if GetComponent is not done on FixedUpdate and the Id is also retrieved on initialization.
                    var trafficLightLaneletId = trafficLights[i].LaneletId;

                    if (distance2D <= radius && trafficLightLaneletId != null && trafficLightLaneletId.relationId != null && trafficLightLaneletId.relationId.Count != 0)
                    {
                        filteredLights.Add(trafficLights[i]);
                    }
                }

                return filteredLights.ToArray();
            }
        }
    }
}