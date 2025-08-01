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

namespace Awsim.Entity
{
    public class AccelVehicleSimpleInputter : MonoBehaviour
    {
        [SerializeField] Component _inputComponent;
        [SerializeField] AccelVehicle _vehicle;
        IAccelVehicleInput _vehicleInput;

        public void Initialize()
        {
            _vehicleInput = _inputComponent as IAccelVehicleInput;
        }

        public void OnUpdate()
        {
            _vehicleInput.UpdateInputs();
        }

        public void OnFixedUpdate()
        {
            _vehicle.AccelerationInput = _vehicleInput.AccelerationInput;
            _vehicle.SteerTireAngleInput = _vehicleInput.SteerAngleInput;
            _vehicle.GearInput = _vehicleInput.GearInput;
            _vehicle.TurnIndicatorsInput = _vehicleInput.TurnIndicatorsInput;
            _vehicle.HazardLightsInput = _vehicleInput.HazardLightsInput;
        }
    }
}