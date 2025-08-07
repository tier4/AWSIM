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
using UnityEngine.Profiling;

namespace Awsim.Usecase.TrafficSimulation
{
    /// <summary>
    /// This class simulates states of NPC vehicles and updates visualization via <see cref="TrafficSimNpcVehicle"/>.
    /// The process of the simulation consists of three steps:<br/>
    /// - Cognition step implemented in <see cref="NpcVehicleCognitionStep"/><br/>
    /// - Decision step implemented in <see cref="NpcVehicleDecisionStep"/><br/>
    /// - Control step implemented in <see cref="NpcVehicleControlStep"/><br/>
    /// Each step updates <see cref="NpcVehicleInternalState"/> and the result is passed to <see cref="NpcVehicleVisualizationStep"/> for visualization update.
    /// </summary>
    public class NpcVehicleSimulator : IDisposable
    {

        /// <summary>
        /// Get NPC vehicle states that are updated in simulation steps.<br/>
        /// </summary>
        public IReadOnlyList<NpcVehicleInternalState> VehicleStates
            => _vehicleStates;

        /// <summary>
        /// Get or set EGO Vehicle that should be considered in the simulation.
        /// </summary>
        public Transform EgoVehicle { get; private set; }
        public int MaxVehicleCount;

        List<NpcVehicleInternalState> _vehicleStates;
        NpcVehicleCognitionStep _cognitionStep;
        NpcVehicleDecisionStep _decisionStep;
        NpcVehicleControlStep _controlStep;
        NpcVehicleVisualizationStep _visualizationStep;
        Transform _dummyEgo;

        public NpcVehicleSimulator(NpcVehicleConfig config,
            LayerMask obstacleLayerMask,
            LayerMask groundLayerMask,
            int maxVehicleCount,
            Transform egoVehicle)
        {
            _vehicleStates = new List<NpcVehicleInternalState>();
            _cognitionStep = new NpcVehicleCognitionStep(obstacleLayerMask, groundLayerMask, maxVehicleCount);
            _decisionStep = new NpcVehicleDecisionStep(config);
            _controlStep = new NpcVehicleControlStep(config);
            _visualizationStep = new NpcVehicleVisualizationStep();
            this.MaxVehicleCount = maxVehicleCount;
            EgoVehicle = egoVehicle;
        }

        /// <summary>
        /// Register <see cref="TrafficSimNpcVehicle"/> to be updated by the simulator.
        /// </summary>
        /// <param name="vehicle">The vehicle to be registered</param>
        /// <param name="lane">Initial lane of the vehicle</param>
        /// <param name="waypointIndex">Current waypoint index of the vehicle</param>
        public void Register(TrafficSimNpcVehicle vehicle, TrafficLane lane, int waypointIndex)
        {
            _vehicleStates.Add(NpcVehicleInternalState.Create(vehicle, lane, waypointIndex));
        }

        /// <summary>
        /// Register <see cref="TrafficSimNpcVehicle"/> to be updated by the simulator.
        /// </summary>
        /// <param name="vehicle">The vehicle to be registered</param>
        /// <param name="route">Route for vehicle to follow</param>
        /// <param name="waypointIndex">Current waypoint index of the vehicle</param>
        public void Register(TrafficSimNpcVehicle vehicle, List<TrafficLane> route, int waypointIndex)
        {
            _vehicleStates.Add(NpcVehicleInternalState.Create(vehicle, route, waypointIndex));
        }

        /// <summary>
        /// This should be called every time any vehicle is destroyed.
        /// </summary>
        public void RemoveInvalidVehicles()
        {
            (VehicleStates as List<NpcVehicleInternalState>)?.RemoveAll(IsVehicleNull);

            // check if vehicle is null
            bool IsVehicleNull(NpcVehicleInternalState state)
            {
                return state.Vehicle == null;
            }
        }

        /// <summary>
        /// Execute simulation steps and update visualization.
        /// </summary>
        /// <param name="deltaTime">Simulation time step</param>
        public void StepOnce(float deltaTime)
        {
            // Simulation steps
            Profiler.BeginSample("NPCVehicleSimulator.Cognition");
            _cognitionStep.Execute(_vehicleStates, EgoVehicle);
            Profiler.EndSample();

            Profiler.BeginSample("NPCVehicleSimulator.Decision");
            _decisionStep.Execute(VehicleStates);
            Profiler.EndSample();

            Profiler.BeginSample("NPCVehicleSimulator.Control");
            _controlStep.Execute(VehicleStates, deltaTime);
            Profiler.EndSample();

            // Visualization step
            Profiler.BeginSample("NPCVehicleSimulator.Visualize");
            _visualizationStep.Execute(VehicleStates);
            Profiler.EndSample();
        }

        /// <summary>
        /// Show editor gizmos for debugging.
        /// </summary>
        public void ShowGizmos(bool showYieldingPhase, bool showObstacleChecking)
        {
            _decisionStep.ShowGizmos(VehicleStates);
            _cognitionStep.ShowGizmos(VehicleStates, showYieldingPhase, showObstacleChecking);
        }

        public void ClearAll()
        {
            foreach (var state in VehicleStates)
            {
                state.ShouldDespawn = true;
            }
        }

        public void Dispose()
        {
            _cognitionStep?.Dispose();
        }
    }
}
