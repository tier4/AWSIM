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

using UnityEngine;
using Awsim.Usecase.TrafficSimulation;
using TMPro;
using System;
using System.Collections;

namespace Awsim.UI
{
    public class TrafficSettingsUIWindow : UIWindow
    {
        public TrafficSimulator TrafficSimulator { get; set; }

        [SerializeField] TMP_InputField _seedInputField = null;
        [SerializeField] TMP_InputField _maxVehicleCountInputField = null;
        [SerializeField] GameObject _appliedTextGameObject = null;

        int _seed = 0;
        int _macVehicleCount = 0;

        public override void OnStart()
        {
            _seed = TrafficSimulator.Seed;
            _macVehicleCount = TrafficSimulator.MaxVehicleCount;

            _seedInputField.text = _seed.ToString();

            _maxVehicleCountInputField.text = _macVehicleCount.ToString();
        }

        public void RestartTrafficSimulation()
        {
            if (_seedInputField.text == string.Empty)
                _seedInputField.text = _seed.ToString();

            if (_maxVehicleCountInputField.text == string.Empty)
                _maxVehicleCountInputField.text = _macVehicleCount.ToString();

            _seed = Int32.Parse(_seedInputField.text);
            _macVehicleCount = Int32.Parse(_maxVehicleCountInputField.text);

            TrafficSimulator.Restart(_seed, _macVehicleCount);

            StartCoroutine(ShowAppriedText());

            IEnumerator ShowAppriedText()
            {
                _appliedTextGameObject.SetActive(true);
                yield return new WaitForSecondsRealtime(3f);
                _appliedTextGameObject.SetActive(false);
            }
        }
    }
}