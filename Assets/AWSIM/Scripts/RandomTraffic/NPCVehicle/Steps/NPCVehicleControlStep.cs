using System;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM.TrafficSimulation
{
    /// <summary>
    /// Control step implementation for a NPC vehicle simulation.
    /// Based on the results of <see cref="NPCVehicleDecisionStep"/>, it outputs linear speed, angular speed, position and rotation of vehicles.
    /// </summary>
    public class NPCVehicleControlStep
    {
        private NPCVehicleConfig config;

        public NPCVehicleControlStep(NPCVehicleConfig config)
        {
            this.config = config;
        }

        public void Execute(IReadOnlyList<NPCVehicleInternalState> states, float deltaTime)
        {
            foreach (var state in states)
            {
                UpdateSpeed(state, deltaTime);
                UpdatePose(state, deltaTime);
                UpdateYawSpeed(state, deltaTime);
            }
        }

        /// <summary>
        /// Update <see cref="NPCVehicleInternalState.Speed"/> according to <see cref="NPCVehicleInternalState.SpeedMode"/>.
        /// </summary>
        private void UpdateSpeed(NPCVehicleInternalState state, float deltaTime)
        {
            if (state.ShouldDespawn)
                return;

            float targetSpeed;
            float acceleration;
            switch (state.SpeedMode)
            {
                case NPCVehicleSpeedMode.NORMAL:
                    targetSpeed = state.CurrentFollowingLane.SpeedLimit;
                    acceleration = config.Acceleration;
                    break;
                case NPCVehicleSpeedMode.SLOW:
                    targetSpeed = Mathf.Min(NPCVehicleConfig.SlowSpeed, state.CurrentFollowingLane.SpeedLimit);
                    acceleration = config.Deceleration;
                    break;
                case NPCVehicleSpeedMode.SUDDEN_STOP:
                    targetSpeed = 0f;
                    acceleration = config.SuddenDeceleration;
                    break;
                case NPCVehicleSpeedMode.ABSOLUTE_STOP:
                    targetSpeed = 0f;
                    acceleration = config.AbsoluteDeceleration;
                    break;
                case NPCVehicleSpeedMode.STOP:
                    targetSpeed = 0f;
                    acceleration = config.Deceleration;
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }

            state.Speed = Mathf.MoveTowards(state.Speed, targetSpeed, acceleration * deltaTime);
        }

        /// <summary>
        /// Update <see cref="NPCVehicleInternalState.YawSpeed"/> according to <see cref="NPCVehicleInternalState.TargetPoint"/>.
        /// </summary>
        private static void UpdateYawSpeed(NPCVehicleInternalState state, float deltaTime)
        {
            // Steering the vehicle so that it heads toward the target point.
            var steeringDirection = state.TargetPoint - state.FrontCenterPosition;
            steeringDirection.y = 0f;
            var steeringAngle = Vector3.SignedAngle(state.Forward, steeringDirection, Vector3.up);
            var targetYawSpeed = steeringAngle * state.Speed * NPCVehicleConfig.YawSpeedMultiplier;
            // Change YawSpeed gradually to eliminate steering shake.
            state.YawSpeed = Mathf.Lerp(
                state.YawSpeed,
                targetYawSpeed,
                NPCVehicleConfig.YawSpeedLerpFactor * deltaTime);
        }

        /// <summary>
        /// Update <see cref="NPCVehicleInternalState.Position"/> and <see cref="NPCVehicleInternalState.Yaw"/> according to <see cref="NPCVehicleInternalState.Speed"/> and <see cref="NPCVehicleInternalState.YawSpeed"/>.
        /// </summary>
        private static void UpdatePose(NPCVehicleInternalState state, float deltaTime)
        {
            if (state.ShouldDespawn)
                return;

            state.Yaw += state.YawSpeed * deltaTime;
            var position = state.Position;
            position += state.Forward * state.Speed * deltaTime;
            position.y = state.TargetPoint.y;
            state.Position = position;
        }
    }
}
