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
    public interface IReadOnlyAccelVehicle
    {
        public float AccelerationInput { get; }
        public float SteerTireAngleInput { get; }
        public Gear GearInput { get; }
        public TurnIndicators TurnIndicatorsInput { get; }
        public HazardLights HazardLightsInput { get; }
        public Vector3 LocalAcceleration { get; }
        public float SteerTireAngle { get; }
        public Gear Gear { get; }
        public TurnIndicators TurnIndicators { get; }
        public HazardLights HazardLights { get; }
        public float Speed { get; }
        public Vector3 Velocity { get; }
        public Vector3 LocalVelocity { get; }
        public Vector3 AngularVelocity { get; }
        public Vector3 AngularAcceleration { get; }
        public AccelVehicle.Wheel[] Wheels { get; }
        public float SteerTireAngleNormalized { get; }
        public float MaxSteerTireAngleInput { get; }
        public float MaxAccelerationInput { get; }
        public float MaxDecelerationInput { get; }
    }
}