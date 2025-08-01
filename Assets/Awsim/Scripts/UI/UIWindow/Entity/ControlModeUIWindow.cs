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

using Awsim.Entity;
using TMPro;
using UnityEngine;

namespace Awsim.UI
{
    public class ControlModeUIWindow : UIWindow
    {
        public AccelVehicleControlModeBasedInputter ControlModeBasedInputProvider
        {
            get => _controlModeBasedInputProvider;
            set
            {
                _controlModeBasedInputProvider = value;
                if (_initialize)
                    SetControlModeByProvider();
            }
        }

        [SerializeField] TMP_Dropdown _dropdown = null;
        AccelVehicleControlModeBasedInputter _controlModeBasedInputProvider = null;
        bool _initialize = false;

        public override void OnStart()
        {
            if (ControlModeBasedInputProvider == null)
                return;

            _dropdown.onValueChanged.AddListener(x =>
            {
                var newControlMode = IntToControlMode(x);

                if (ControlModeBasedInputProvider.ControlMode != newControlMode)
                    ControlModeBasedInputProvider.ControlMode = newControlMode;
            });

            _initialize = true;

            SetControlModeByProvider();
        }

        public override void OnUpdate()
        {
            SetControlModeByProvider();
        }

        void SetControlModeByProvider()
        {
            var currentControlMode = _controlModeBasedInputProvider.ControlMode;
            var dropdownValue = ControlModeToInt(currentControlMode);
            if (_dropdown.value == dropdownValue)
                return;

            _dropdown.value = dropdownValue;
        }

        static ControlMode IntToControlMode(int value)
        {
            if (value == 0)
                return ControlMode.Autonomous;
            else
                return ControlMode.Manual;
        }

        static int ControlModeToInt(ControlMode controlMode)
        {
            if (controlMode == ControlMode.Autonomous)
                return 0;
            else
                return 1;
        }
    }
}