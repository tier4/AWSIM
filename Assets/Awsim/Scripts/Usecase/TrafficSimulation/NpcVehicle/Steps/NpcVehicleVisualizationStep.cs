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
using Awsim.Entity;

namespace Awsim.Usecase.TrafficSimulation
{
    /// <summary>
    /// Visualization step implementation for a NPC vehicle simulation.
    /// Based on the results of the simulation steps, it updates <see cref="TrafficSimNpcVehicle"/>.
    /// </summary>
    public class NpcVehicleVisualizationStep
    {
        public void Execute(IReadOnlyList<NpcVehicleInternalState> states)
        {
            foreach (var state in states)
            {
                ApplyPose(state);
                ApplyTurnSignalState(state);
            }
        }

        static void ApplyPose(NpcVehicleInternalState state)
        {
            if (state.ShouldDespawn)
                return;

            var vehicle = state.Vehicle;
            vehicle.NpcVehicle.PoseInput = new Pose(state.Position, Quaternion.AngleAxis(state.Yaw, Vector3.up));
        }

        static void ApplyTurnSignalState(NpcVehicleInternalState state)
        {
            if (state.ShouldDespawn || state.FollowingLanes.Count <= 1)
                return;

            var vehicle = state.Vehicle;
            switch (state.FollowingLanes[0].TurnDirection)
            {
                case TrafficLane.TurnDirectionType.Left:
                    vehicle.NpcVehicle.SetTurnSignalState(PoseVehicle.TurnSignalState.Left);
                    break;
                case TrafficLane.TurnDirectionType.Right:
                    vehicle.NpcVehicle.SetTurnSignalState(PoseVehicle.TurnSignalState.Right);
                    break;
                case TrafficLane.TurnDirectionType.Straight:
                default:
                    switch (state.FollowingLanes[1].TurnDirection)
                    {
                        case TrafficLane.TurnDirectionType.Left:
                            vehicle.NpcVehicle.SetTurnSignalState(PoseVehicle.TurnSignalState.Left);
                            break;
                        case TrafficLane.TurnDirectionType.Right:
                            vehicle.NpcVehicle.SetTurnSignalState(PoseVehicle.TurnSignalState.Right);
                            break;
                        case TrafficLane.TurnDirectionType.Straight:
                        default:
                            vehicle.NpcVehicle.SetTurnSignalState(PoseVehicle.TurnSignalState.Off);
                            break;
                    }

                    break;
            }
        }
    }
}
