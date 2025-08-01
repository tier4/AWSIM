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

using System.Linq;
using Awsim.Entity;
using TMPro;
using UnityEngine;

namespace Awsim.UI
{
    public class AccelVehicleInputDeviceUIWindow : UIWindow
    {
        public AccelVehicleControlModeBasedInputter ControlModeBasedInputter
        {
            get => _controlModeBasedInputter;
            set
            {
                _controlModeBasedInputter = value;
                if (_initialized)
                {
                    SetDropdownOptionsByProvider();
                    SetDropdownValueByProvider();
                    _dropdown.value = 0;
                }
            }
        }

        [SerializeField] TMP_Dropdown _dropdown = null;
        AccelVehicleControlModeBasedInputter _controlModeBasedInputter = null;

        bool _initialized = false;

        public override void OnStart()
        {
            if (ControlModeBasedInputter != null)
            {
                SetDropdownOptionsByProvider();
                SetDropdownValueByProvider();
            }

            _dropdown.onValueChanged.AddListener(x =>
            {
                _controlModeBasedInputter.SetActiveManuallyInputIndex(x);
            });

            _initialized = true;
        }

        void SetDropdownOptionsByProvider()
        {
            _dropdown.ClearOptions();
            var optionDataList = ControlModeBasedInputter.ManuallyInputs.Select(x => new TMP_Dropdown.OptionData(x.Name)).ToList();
            _dropdown.AddOptions(optionDataList);
        }

        void SetDropdownValueByProvider()
        {
            _dropdown.value = _controlModeBasedInputter.ActiveManuallyInputIndex;
        }
    }
}