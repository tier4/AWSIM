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
using TMPro;
using Awsim.Entity;

namespace Awsim.UI
{
    public class AccelVehicleInformationUIWindow : UIWindow
    {
        public IReadOnlyAccelVehicle Vehicle { get; set; }
        public AccelVehicleControlModeBasedInputter ControlModeBasedInputter { get; set; }
        [SerializeField] TMP_Text _text;

        public override void OnUpdate()
        {
            if (Shown == false)
                return;

            string str = "";

            if (Vehicle == null)
                str = "none vehicle";

            str = "--- Input --- \n"
                + "Acceleration input (m/s^2)    : " + Vehicle.AccelerationInput.ToString("F2") + "\n"
                + "Steer tire angle input (deg.) : " + Vehicle.SteerTireAngleInput.ToString("F2") + "\n\n"
                + "--- State --- \n"
                + "Acceleration (m/s^2)          : " + Vehicle.LocalAcceleration.z.ToString("F2") + "\n"
                + "Steer tire angle (deg.)       : " + Vehicle.SteerTireAngle.ToString("F2") + "\n"
                + "Gear                          : " + Vehicle.Gear + "\n"
                + "Turn indicators               : " + Vehicle.TurnIndicators + "\n"
                + "Hazard lights                 : " + Vehicle.HazardLights + "\n"
                + "Control mode                  : " + ControlModeBasedInputter.ControlMode + "\n"
                + "Speed (m/s)                   : " + Vehicle.Speed + "\n";

            _text.text = str;
        }
    }
}