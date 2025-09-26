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



namespace Awsim.Entity
{
    public interface IAccelVehicleInput
    {
        public string Name { get; }

        public float AccelerationInput { get; }

        public float SteerAngleInput { get; }

        public Gear GearInput { get; }

        public TurnIndicators TurnIndicatorsInput { get; }

        public HazardLights HazardLightsInput { get; }

        public bool SwitchAutonomous { get; }

        public bool UpdateInputs();     // TODO: It might be better to return overridden controls with enum instead of bool.
    }
}