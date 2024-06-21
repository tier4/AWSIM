using System.Collections.Generic;
using UnityEngine;

namespace AWSIM.TrafficSimulation
{
    /// <summary>
    /// Visualization step implementation for a NPC vehicle simulation.
    /// Based on the results of the simulation steps, it updates <see cref="NPCVehicle"/>.
    /// </summary>
    public class NPCVehicleVisualizationStep
    {
        public void Execute(IReadOnlyList<NPCVehicleInternalState> states, Transform egoVehicle)
        {
            foreach (var state in states)
            {
                ApplyPose(state);
                ApplyTurnSignalState(state);
            }
        }

        private static void ApplyPose(NPCVehicleInternalState state)
        {
            if (state.ShouldDespawn)
                return;

            var vehicle = state.Vehicle;
            vehicle.SetPosition(state.Position);
            vehicle.SetRotation(Quaternion.AngleAxis(state.Yaw, Vector3.up));
        }

        private static void ApplyTurnSignalState(NPCVehicleInternalState state)
        {
            if (state.ShouldDespawn || state.FollowingLanes.Count <= 1)
                return;

            var vehicle = state.Vehicle;
            switch (state.FollowingLanes[0].TurnDirection)
            {
                case TrafficLane.TurnDirectionType.LEFT:
                    vehicle.SetTurnSignalState(NPCVehicle.TurnSignalState.LEFT);
                    break;
                case TrafficLane.TurnDirectionType.RIGHT:
                    vehicle.SetTurnSignalState(NPCVehicle.TurnSignalState.RIGHT);
                    break;
                case TrafficLane.TurnDirectionType.STRAIGHT:
                default:
                    switch (state.FollowingLanes[1].TurnDirection)
                    {
                        case TrafficLane.TurnDirectionType.LEFT:
                            vehicle.SetTurnSignalState(NPCVehicle.TurnSignalState.LEFT);
                            break;
                        case TrafficLane.TurnDirectionType.RIGHT:
                            vehicle.SetTurnSignalState(NPCVehicle.TurnSignalState.RIGHT);
                            break;
                        case TrafficLane.TurnDirectionType.STRAIGHT:
                        default:
                            vehicle.SetTurnSignalState(NPCVehicle.TurnSignalState.OFF);
                            break;
                    }

                    break;
            }
        }
    }
}
