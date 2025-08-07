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
using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace Awsim.UI
{
    public class ToggleButton : MonoBehaviour
    {
        [SerializeField] Toggle _toggle;
        [SerializeField] TMP_Text _text;

        public void Initialize(string windowName, bool initialShown, Action toggleOnAction, Action toggleOffAction)
        {
            _toggle.isOn = initialShown;

            _text.text = windowName;
            _toggle.onValueChanged.AddListener(isOn =>
            {
                if (isOn)
                    toggleOnAction();
                else
                    toggleOffAction();
            });
        }

        public void SetIsOn(bool isOn)
        {
            _toggle.isOn = isOn;
        }
    }
}