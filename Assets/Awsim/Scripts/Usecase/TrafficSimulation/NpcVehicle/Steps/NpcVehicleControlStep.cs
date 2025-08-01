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

using System;
using System.Collections.Generic;
using UnityEngine;

namespace Awsim.Usecase.TrafficSimulation
{
    /// <summary>
    /// Control step implementation for a NPC vehicle simulation.
    /// Based on the results of <see cref="NpcVehicleDecisionStep"/>, it outputs linear speed, angular speed, position and rotation of vehicles.
    /// </summary>
    public class NpcVehicleControlStep
    {
        NpcVehicleConfig _config;

        public NpcVehicleControlStep(NpcVehicleConfig config)
        {
            this._config = config;
        }

        public void Execute(IReadOnlyList<NpcVehicleInternalState> states, float deltaTime)
        {
            foreach (var state in states)
            {
                UpdateSpeed(state, deltaTime);
                UpdatePose(state, deltaTime);
                UpdateYawSpeed(state, deltaTime);
            }
        }

        /// <summary>
        /// Update <see cref="NpcVehicleInternalState.YawSpeed"/> according to <see cref="NpcVehicleInternalState.TargetPoint"/>.
        /// </summary>
        static void UpdateYawSpeed(NpcVehicleInternalState state, float deltaTime)
        {
            // Steering the vehicle so that it heads toward the target point.
            var steeringDirection = state.TargetPoint - state.FrontCenterPosition;
            steeringDirection.y = 0f;
            var steeringAngle = Vector3.SignedAngle(state.Forward, steeringDirection, Vector3.up);
            var targetYawSpeed = steeringAngle * state.Speed * NpcVehicleConfig.YawSpeedMultiplier;
            // Change YawSpeed gradually to eliminate steering shake.
            state.YawSpeed = Mathf.Lerp(
                state.YawSpeed,
                targetYawSpeed,
                NpcVehicleConfig.YawSpeedLerpFactor * deltaTime);
        }

        /// <summary>
        /// Update <see cref="NpcVehicleInternalState.Position"/> and <see cref="NpcVehicleInternalState.Yaw"/> according to <see cref="NpcVehicleInternalState.Speed"/> and <see cref="NpcVehicleInternalState.YawSpeed"/>.
        /// </summary>
        static void UpdatePose(NpcVehicleInternalState state, float deltaTime)
        {
            if (state.ShouldDespawn)
                return;

            state.Yaw += state.YawSpeed * deltaTime;
            var position = state.Position;
            position += state.Forward * state.Speed * deltaTime;
            position.y = state.TargetPoint.y;
            state.Position = position;
        }

        /// <summary>
        /// Update <see cref="NpcVehicleInternalState.Speed"/> according to <see cref="NpcVehicleInternalState.SpeedMode"/>.
        /// </summary>
        void UpdateSpeed(NpcVehicleInternalState state, float deltaTime)
        {
            if (state.ShouldDespawn)
                return;

            float targetSpeed;
            float acceleration;
            switch (state.SpeedMode)
            {
                case NpcVehicleSpeedMode.Normal:
                    targetSpeed = state.CurrentFollowingLane.SpeedLimit;
                    acceleration = _config.Acceleration;
                    break;
                case NpcVehicleSpeedMode.Slow:
                    targetSpeed = Mathf.Min(NpcVehicleConfig.SlowSpeed, state.CurrentFollowingLane.SpeedLimit);
                    acceleration = _config.Deceleration;
                    break;
                case NpcVehicleSpeedMode.SuddenStop:
                    targetSpeed = 0f;
                    acceleration = _config.SuddenDeceleration;
                    break;
                case NpcVehicleSpeedMode.AbsoluteStop:
                    targetSpeed = 0f;
                    acceleration = _config.AbsoluteDeceleration;
                    break;
                case NpcVehicleSpeedMode.Stop:
                    targetSpeed = 0f;
                    acceleration = _config.Deceleration;
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }

            state.Speed = Mathf.MoveTowards(state.Speed, targetSpeed, acceleration * deltaTime);
        }
    }
}
