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
    public class AccelVehicleMeterUIWindow : UIWindow
    {
        public AccelVehicle Vehicle { get; set; }

        [SerializeField] TMP_Text _speedValueText;
        [SerializeField] TMP_Text _gearValueText;

        public override void OnStart()
        {
            UpdateText();
        }

        public override void OnUpdate()
        {
            UpdateText();
        }

        void UpdateText()
        {
            if (Vehicle == null || !Shown)
                return;

            _speedValueText.text = (Vehicle.Speed * 3.6f).ToString("F0");

            if (Vehicle.Gear == Gear.Drive)
                _gearValueText.text = "D";
            else if (Vehicle.Gear == Gear.Neutral)
                _gearValueText.text = "N";
            else if (Vehicle.Gear == Gear.Parking)
                _gearValueText.text = "P";
            else if (Vehicle.Gear == Gear.Reverse)
                _gearValueText.text = "R";
        }
    }
}