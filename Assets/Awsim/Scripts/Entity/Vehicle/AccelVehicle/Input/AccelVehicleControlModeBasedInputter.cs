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
using UnityEngine;

namespace Awsim.Entity
{
    public class AccelVehicleControlModeBasedInputter : MonoBehaviour
    {
        public ControlMode ControlMode { get => _controlMode; set => _controlMode = value; }
        public IAccelVehicleInput[] ManuallyInputs { get => _manuallyInputs; }
        public int ActiveManuallyInputIndex { get => _activeManuallyInputIndex; }

        bool _hasManuallyInputs => _manuallyInputs.Length > 0;

        [SerializeField] ControlMode _controlMode = ControlMode.Autonomous;
        [SerializeField] Component _autonomousInputComponent = null;
        [SerializeField] Component[] _manuallyInputComponents = null;
        [SerializeField] int _activeManuallyInputIndex = 0;
        [SerializeField] AccelVehicle _vehicle = null;

        IAccelVehicleInput _autonomousInput = null;
        IAccelVehicleInput[] _manuallyInputs = null;
        bool _isOverridden;
        bool _isSwitchAutonomous;

        public void SetActiveManuallyInputIndex(int index)
        {
            _activeManuallyInputIndex = index;
        }

        public void Initialize()
        {
            _autonomousInput = _autonomousInputComponent as IAccelVehicleInput;
            _manuallyInputs = _manuallyInputComponents.Select(x => x as IAccelVehicleInput).ToArray();
        }

        public void OnUpdate()
        {
            _autonomousInput.UpdateInputs();     // NOTE: Might be more stable to use FixedUpdate() to get input from Ros2. Need to investigate.

            if (_hasManuallyInputs)
            {
                for (int i = 0; i < ManuallyInputs.Length; i++)
                {

                    var tempOverriden = ManuallyInputs[i].UpdateInputs();
                    if (i == _activeManuallyInputIndex)
                        _isOverridden = tempOverriden;
                }
            }
        }

        public void OnFixedUpdate()
        {
            if (_isOverridden && ControlMode == ControlMode.Autonomous)
                ControlMode = ControlMode.Manual;

            // TODO: implement NO_COMMAND, DISENGAGED, NOT_READY, AUTONOMOUS_STEER_ONLY, AUTONOMOUS_VELOCITY_ONLY

            if (ControlMode == ControlMode.Autonomous)
            {
                _vehicle.AccelerationInput = _autonomousInput.AccelerationInput;
                _vehicle.SteerTireAngleInput = _autonomousInput.SteerAngleInput;
                _vehicle.GearInput = _autonomousInput.GearInput;
                _vehicle.TurnIndicatorsInput = _autonomousInput.TurnIndicatorsInput;
                _vehicle.HazardLightsInput = _autonomousInput.HazardLightsInput;
            }
            else if (ControlMode == ControlMode.Manual)
            {
                if (!_hasManuallyInputs)
                    return;

                var manuallyInput = ManuallyInputs[_activeManuallyInputIndex];

                if (manuallyInput.SwitchAutonomous)
                    ControlMode = ControlMode.Autonomous;

                _vehicle.AccelerationInput = manuallyInput.AccelerationInput;
                _vehicle.SteerTireAngleInput = manuallyInput.SteerAngleInput;
                _vehicle.GearInput = manuallyInput.GearInput;
                _vehicle.TurnIndicatorsInput = manuallyInput.TurnIndicatorsInput;
                _vehicle.HazardLightsInput = manuallyInput.HazardLightsInput;
            }
        }
    }
}