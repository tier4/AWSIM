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

namespace Awsim.Usecase.TrafficSimulation
{
    /// <summary>
    /// Config of vehicle behaviors in <see cref="NpcVehicleSimulator"/>.
    /// </summary>
    [System.Serializable]
    public struct NpcVehicleConfig
    {
        /// <summary>
        /// Multiplier used to determine rotational speed from steering angle and vehicle speed.
        /// </summary>
        public const float YawSpeedMultiplier = 0.15f;

        /// <summary>
        /// Rate of change of angular velocity per unit time.<br/>
        /// The higher the value, the faster it can turn, but the more blurred the control becomes.
        /// </summary>
        public const float YawSpeedLerpFactor = 5f;

        /// <summary>
        /// Slow speed at which the vehicle can immediately stop.
        /// </summary>
        public const float SlowSpeed = 5f;

        public float Acceleration;
        public float Deceleration;
        public float SuddenDeceleration;
        public float AbsoluteDeceleration;

        public static NpcVehicleConfig Default()
            => new NpcVehicleConfig
            {
                Acceleration = 3f,
                Deceleration = 2f,
                SuddenDeceleration = 4f,
                AbsoluteDeceleration = 20f
            };
    }
}
