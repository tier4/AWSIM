using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Profiling;

namespace AWSIM.TrafficSimulation
{
    /// <summary>
    /// This class simulates states of NPC vehicles and updates visualization via <see cref="NPCVehicle"/>.
    /// The process of the simulation consists of three steps:<br/>
    /// - Cognition step implemented in <see cref="NPCVehicleCognitionStep"/><br/>
    /// - Decision step implemented in <see cref="NPCVehicleDecisionStep"/><br/>
    /// - Control step implemented in <see cref="NPCVehicleControlStep"/><br/>
    /// Each step updates <see cref="NPCVehicleInternalState"/> and the result is passed to <see cref="NPCVehicleVisualizationStep"/> for visualization update.
    /// </summary>
    public class NPCVehicleSimulator : IDisposable
    {

        /// <summary>
        /// Get NPC vehicle states that are updated in simulation steps.<br/>
        /// </summary>
        public IReadOnlyList<NPCVehicleInternalState> VehicleStates
            => vehicleStates;

        /// <summary>
        /// Get or set EGO Vehicle that should be considered in the simulation.
        /// </summary>
        public Transform EGOVehicle { get; set; }
        public int maxVehicleCount;

        private List<NPCVehicleInternalState> vehicleStates;
        private NPCVehicleCognitionStep cognitionStep;
        private NPCVehicleDecisionStep decisionStep;
        private NPCVehicleControlStep controlStep;
        private NPCVehicleVisualizationStep visualizationStep;
        private Transform dummyEgo;

        public NPCVehicleSimulator(NPCVehicleConfig config,
            LayerMask vehicleLayerMask,
            LayerMask groundLayerMask,
            int maxVehicleCount,
            GameObject egoVehicle) 
        {
            vehicleStates = new List<NPCVehicleInternalState>();
            cognitionStep = new NPCVehicleCognitionStep(vehicleLayerMask, groundLayerMask, maxVehicleCount);
            decisionStep = new NPCVehicleDecisionStep(config);
            controlStep = new NPCVehicleControlStep(config);
            visualizationStep = new NPCVehicleVisualizationStep();
            this.maxVehicleCount = maxVehicleCount;
            EGOVehicle = egoVehicle.transform; 
        }


        /// <summary>
        /// When there is no real Ego vehicle in a scene, dummy one must be set.
        /// </summary>
        public void UnregisterEgo()
        {
            if(dummyEgo)
            {
                EGOVehicle = dummyEgo;
            }
        }

        /// <summary>
        /// Dummy ego is used when there is no real Ego in the scene
        /// </summary>
        public void SetDummyEgo(GameObject ego)
        {
            dummyEgo = ego.transform;
        }
        
        /// <summary>
        /// Registers Ego vehicle.
        /// </summary>
        public void RegisterEgo(GameObject egoVehicle)
        {
            EGOVehicle = egoVehicle.transform;
        }

        /// <summary>
        /// Register <see cref="NPCVehicle"/> to be updated by the simulator.
        /// </summary>
        /// <param name="vehicle">The vehicle to be registered</param>
        /// <param name="lane">Initial lane of the vehicle</param>
        /// <param name="waypointIndex">Current waypoint index of the vehicle</param>
        public void Register(NPCVehicle vehicle, TrafficLane lane, int waypointIndex)
        {
            vehicleStates.Add(NPCVehicleInternalState.Create(vehicle, lane, waypointIndex));
        }

        /// <summary>
        /// Register <see cref="NPCVehicle"/> to be updated by the simulator.
        /// </summary>
        /// <param name="vehicle">The vehicle to be registered</param>
        /// <param name="route">Route for vehicle to follow</param>
        /// <param name="waypointIndex">Current waypoint index of the vehicle</param>
        public void Register(NPCVehicle vehicle, List<TrafficLane> route, int waypointIndex)
        {
            vehicleStates.Add(NPCVehicleInternalState.Create(vehicle, route, waypointIndex));
        }

        /// <summary>
        /// This should be called every time any vehicle is destroyed.
        /// </summary>
        public void RemoveInvalidVehicles()
        {
            (VehicleStates as List<NPCVehicleInternalState>)?.RemoveAll(IsVehicleNull);

            // check if vehicle is null
            bool IsVehicleNull(NPCVehicleInternalState state)
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
            cognitionStep.Execute(vehicleStates, EGOVehicle);
            Profiler.EndSample();

            Profiler.BeginSample("NPCVehicleSimulator.Decision");
            decisionStep.Execute(VehicleStates);
            Profiler.EndSample();

            Profiler.BeginSample("NPCVehicleSimulator.Control");
            controlStep.Execute(VehicleStates, deltaTime);
            Profiler.EndSample();

            // Visualization step
            Profiler.BeginSample("NPCVehicleSimulator.Visualize");
            visualizationStep.Execute(VehicleStates, EGOVehicle);
            Profiler.EndSample();
        }

        /// <summary>
        /// Show editor gizmos for debugging.
        /// </summary>
        public void ShowGizmos()
        {
            decisionStep.ShowGizmos(VehicleStates);
            cognitionStep.ShowGizmos(VehicleStates);
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
            cognitionStep?.Dispose();
        }
    }
}
