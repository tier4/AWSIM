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

using System.Collections.Generic;
using UnityEngine;

namespace Awsim.Usecase.TrafficSimulation
{
    /// <summary>
    /// Decision step implementation for a NPC vehicle simulation.
    /// Based on the results of the cognitive step, it outputs a short-term target point and a decision to decelerate or accelerate.
    /// </summary>
    public class NpcVehicleDecisionStep
    {
        NpcVehicleConfig _config;

        // MinFrontVehicleDistance is added to the threshold for the distance at which an obstacle is considered dangerous.
        // The vehicle is controlled to stop at this distance away from the obstacle(e.g. another vehicle in front of the vehicle).
        const float _minFrontVehicleDistance = 4f;
        const float _minStopDistance = 1.5f;

        public NpcVehicleDecisionStep(NpcVehicleConfig config)
        {
            this._config = config;
        }

        public void Execute(IReadOnlyList<NpcVehicleInternalState> states)
        {
            foreach (var state in states)
            {
                UpdateTargetPoint(state);
                UpdateSpeedMode(state, _config);
            }
        }

        public void ShowGizmos(IReadOnlyList<NpcVehicleInternalState> states)
        {
            foreach (var state in states)
            {
                switch (state.SpeedMode)
                {
                    case NpcVehicleSpeedMode.AbsoluteStop:
                    case NpcVehicleSpeedMode.SuddenStop:
                    case NpcVehicleSpeedMode.Stop:
                        Gizmos.color = Color.red;
                        break;
                    case NpcVehicleSpeedMode.Slow:
                        Gizmos.color = Color.yellow;
                        break;
                    default:
                        Gizmos.color = Color.green;
                        break;
                }

                var currentPosition = state.FrontCenterPosition;
                currentPosition.y += 1f;

                Gizmos.DrawLine(currentPosition, state.TargetPoint);

                var rotation = Quaternion.LookRotation(currentPosition - state.TargetPoint);
                Gizmos.matrix = Matrix4x4.TRS(state.TargetPoint, rotation, Vector3.one);
                Gizmos.DrawFrustum(Vector3.zero, 30f, 1f, 0f, 1f);
                Gizmos.matrix = Matrix4x4.identity;
            }
        }

        /// <summary>
        /// Set short-term target point of the vehicle to the next waypoint.
        /// </summary>
        /// <param name="state"></param>
        static void UpdateTargetPoint(NpcVehicleInternalState state)
        {
            if (state.ShouldDespawn || state.CurrentFollowingLane == null)
                return;

            state.TargetPoint = state.CurrentFollowingLane.Waypoints[state.WaypointIndex];
        }

        /// <summary>
        /// Update speed mode according to the following cognition results and stoppable distance calculated by speed.<br/>
        /// Possible speed modes and conditions are the following:<br/>
        /// - SLOW when the vehicle is in a sharp curve, needs to keep distance from a front vehicle or is entering yielding lane.<br/>
        /// - STOP when the vehicle can stop safely at a stop point(e.g. a stop line or a point that an obstacle exists).<br/>
        /// - SUDDEN_STOP when the vehicle cannot stop safely at a stop point.<br/>
        /// - ABSOLUTE_STOP when the vehicle cannot stop using SUDDEN_STOP<br/>
        /// - NORMAL under other conditions.
        // /// </summary>
        static void UpdateSpeedMode(NpcVehicleInternalState state, NpcVehicleConfig config)
        {
            if (state.ShouldDespawn)
            {
                return;
            }

            var absoluteStopDistance = CalculateStoppableDistance(state.Speed, config.AbsoluteDeceleration) + _minStopDistance;
            var suddenStopDistance = CalculateStoppableDistance(state.Speed, config.SuddenDeceleration) + 2 * _minStopDistance;
            var stopDistance = CalculateStoppableDistance(state.Speed, config.Deceleration) + 3 * _minStopDistance;
            var slowDownDistance = stopDistance + 4 * _minStopDistance;

            var distanceToStopPointByFrontVehicle = OnlyGreaterThan(state.DistanceToFrontVehicle - _minFrontVehicleDistance, -_minFrontVehicleDistance);
            var distanceToStopPointByTrafficLight = CalculateTrafficLightDistance(state, suddenStopDistance);
            var distanceToStopPointByRightOfWay = CalculateYieldingDistance(state);
            var distanceToStopPoint = Mathf.Min(distanceToStopPointByFrontVehicle, distanceToStopPointByTrafficLight, distanceToStopPointByRightOfWay);

            state.IsStoppedByFrontVehicle = false;
            if (distanceToStopPointByFrontVehicle <= stopDistance)
            {
                state.IsStoppedByFrontVehicle = true;
            }

            if (distanceToStopPoint <= absoluteStopDistance)
                state.SpeedMode = NpcVehicleSpeedMode.AbsoluteStop;
            else if (distanceToStopPoint <= suddenStopDistance)
                state.SpeedMode = NpcVehicleSpeedMode.SuddenStop;
            else if (distanceToStopPoint <= stopDistance)
                state.SpeedMode = NpcVehicleSpeedMode.Stop;
            else if (distanceToStopPoint <= slowDownDistance || state.IsTurning)
                state.SpeedMode = NpcVehicleSpeedMode.Slow;
            else
                state.SpeedMode = NpcVehicleSpeedMode.Normal;
        }

        static float CalculateTrafficLightDistance(NpcVehicleInternalState state, float suddenStopDistance)
        {
            var distanceToStopPointByTrafficLight = float.MaxValue;
            if (state.TrafficLightLane != null)
            {
                var distanceToStopLine = state.DistanceToClosestTrafficLightLane();
                switch (state.TrafficLightPassability)
                {
                    case TrafficLightPassability.Green:
                        break;
                    case TrafficLightPassability.Yellow:
                        if (distanceToStopLine < suddenStopDistance) break;
                        distanceToStopPointByTrafficLight = distanceToStopLine;
                        break;
                    case TrafficLightPassability.Red:
                        distanceToStopPointByTrafficLight = distanceToStopLine;
                        break;
                }
            }
            return OnlyGreaterThan(distanceToStopPointByTrafficLight, 0);
        }

        static float CalculateYieldingDistance(NpcVehicleInternalState state)
        {
            var distanceToStopPointByRightOfWay = float.MaxValue;
            if (state.YieldPhase != NpcVehicleYieldPhase.None && state.YieldPhase != NpcVehicleYieldPhase.EnteringIntersection && state.YieldPhase != NpcVehicleYieldPhase.AtIntersection)
                distanceToStopPointByRightOfWay = state.SignedDistanceToPointOnLane(state.YieldPoint);
            return OnlyGreaterThan(distanceToStopPointByRightOfWay, -float.MaxValue);
        }

        static float CalculateStoppableDistance(float speed, float deceleration)
        {
            return OnlyGreaterThan(speed * speed / 2f / deceleration, 0);
        }

        static float OnlyGreaterThan(float value, float min_value = 0)
        { return value >= min_value ? value : float.MaxValue; }
    }
}