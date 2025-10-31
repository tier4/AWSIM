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
    /// <summary>
    /// Controlling vehicle input via keyboard.
    /// </summary>
    /// ----- Default key binds -----
    /// up arrow            : Accelerate
    /// down arrow          : Deceleration
    /// left/right arrow    : Steering
    /// D                   : Drive gear
    /// P                   : Parking gear
    /// R                   : Reverse gear
    /// N                   : Neutral gear
    /// 1                   : Left turn signal
    /// 2                   : Right turn signal
    /// 3                   : Hazard
    /// 4                   : Turn signal off
    public class AccelVehicleKeyboardInput : MonoBehaviour, IAccelVehicleInput
    {
        public string Name => "Keyboard";
        public float AccelerationInput { get; private set; }
        public float SteerAngleInput { get; private set; }
        public Gear GearInput { get; private set; } = Gear.Drive;
        public TurnIndicators TurnIndicatorsInput { get; private set; }
        public HazardLights HazardLightsInput { get; private set; }
        public bool SwitchAutonomous { get; private set; }

        [SerializeField] KeyCode _accelerationKey = KeyCode.UpArrow;
        [SerializeField] KeyCode _decelerationKey = KeyCode.DownArrow;
        [SerializeField] KeyCode _leftSteeringKey = KeyCode.LeftArrow;
        [SerializeField] KeyCode _rightSteeringKey = KeyCode.RightArrow;
        [SerializeField] KeyCode _driveGearKey = KeyCode.D;
        [SerializeField] KeyCode _parkingGearKey = KeyCode.P;
        [SerializeField] KeyCode _reverseGearKey = KeyCode.R;
        [SerializeField] KeyCode _neutralGearKey = KeyCode.N;
        [SerializeField] KeyCode _leftTurnSignalKey = KeyCode.Alpha1;
        [SerializeField] KeyCode _rightTurnSignalKey = KeyCode.Alpha2;
        [SerializeField] KeyCode _hazardSignalKey = KeyCode.Alpha3;
        [SerializeField] KeyCode _noneSignalKey = KeyCode.Alpha4;
        [SerializeField] KeyCode _switchAutonomousKey = KeyCode.C;
        [SerializeField] Component _readonlyVehicleComponent = null;
        IReadOnlyAccelVehicle _readonlyVehicle = null;

        public void Initialize()
        {
            _readonlyVehicle = _readonlyVehicleComponent as IReadOnlyAccelVehicle;
        }

        public bool UpdateInputs()
        {
            bool isOverridden = false;


            // Get switch autonomous.
            SwitchAutonomous = false;
            if (Input.GetKey(_switchAutonomousKey))
                SwitchAutonomous = true;

            // Get acceleration.
            if (Input.GetKey(_accelerationKey))
            {
                AccelerationInput = _readonlyVehicle.MaxAccelerationInput;
                isOverridden = true;
            }
            else if (Input.GetKey(_decelerationKey))
            {
                AccelerationInput = _readonlyVehicle.MaxDecelerationInput * -1;
                isOverridden = true;
            }
            else
                AccelerationInput = 0;

            // Get steering.
            if (Input.GetKey(_leftSteeringKey))
            {
                SteerAngleInput = _readonlyVehicle.MaxSteerTireAngleInput * -1;
                isOverridden = true;
            }
            else if (Input.GetKey(_rightSteeringKey))
            {
                SteerAngleInput = _readonlyVehicle.MaxSteerTireAngleInput;
                isOverridden = true;
            }
            else
                SteerAngleInput = 0;

            // Get shift.
            if (Input.GetKey(_driveGearKey))
                GearInput = Gear.Drive;
            else if (Input.GetKey(_parkingGearKey))
                GearInput = Gear.Parking;
            else if (Input.GetKey(_reverseGearKey))
                GearInput = Gear.Reverse;
            else if (Input.GetKey(_neutralGearKey))
                GearInput = Gear.Neutral;
            else
                GearInput = _readonlyVehicle.GearInput;

            // Get turn signal.
            if (Input.GetKey(_leftTurnSignalKey))               // Left turn signal.
                TurnIndicatorsInput = TurnIndicators.Left;
            else if (Input.GetKey(_rightTurnSignalKey))         // Right turn signal.
                TurnIndicatorsInput = TurnIndicators.Right;
            else if (Input.GetKey(_noneSignalKey))              // None.
                TurnIndicatorsInput = TurnIndicators.None;
            else
                TurnIndicatorsInput = _readonlyVehicle.TurnIndicatorsInput;

            // Get hazard signal.
            if (Input.GetKeyDown(_hazardSignalKey))
            {
                if (HazardLightsInput == HazardLights.Enable)
                    HazardLightsInput = HazardLights.Disable;
                else
                    HazardLightsInput = HazardLights.Enable;
            }
            else
                HazardLightsInput = _readonlyVehicle.HazardLightsInput;

            return isOverridden;
        }
    }
}