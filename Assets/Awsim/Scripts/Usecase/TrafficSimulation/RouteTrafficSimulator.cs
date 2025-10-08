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
using System.Linq;
using UnityEngine;

namespace Awsim.Usecase.TrafficSimulation
{
    [Serializable]
    public struct RouteTrafficSimulatorConfiguration
    {
        public bool EnableSimulation => _enableSimulation;

        public TrafficSimNpcVehicle[] TrafficSimNpcVehiclePrefabs => _trafficSimNpcVehiclePrefabs;

        public TrafficLane[] RouteLanes => _routeLanes;

        public bool EnableSpawnCountLimit => _enableSpawnCountLimit;

        public int SpawnCountLimit => _spawnCountLimit;


        [SerializeField]
        bool _enableSimulation;

        [SerializeField]
        TrafficSimNpcVehicle[] _trafficSimNpcVehiclePrefabs;

        [Tooltip("Route to follow. The first element also acts as a spawn lane.")]
        [SerializeField]
        TrafficLane[] _routeLanes;

        [SerializeField]
        bool _enableSpawnCountLimit;

        [Tooltip("Describes the lifetime of a traffic simulator instance by specifying how many vehicles this traffic simulator will spawn. Setting it makes the spawner live longer or shorter, while it can also be set to infinity if needed (endless lifetime).")]
        [SerializeField]
        int _spawnCountLimit;
    }

    /// <summary>
    /// Simulate NPC traffic. (Currently only Vehicle is supported)
    /// - Use provided route
    /// - Continue driving randomly when the route ends 
    /// - Traffic light control based on the Vienna Convention
    /// </summary>
    public class RouteTrafficSimulator : ITrafficSimulator
    {
        public bool enabled = true;

        private TrafficLane[] route;
        private int maximumSpawns = 0;
        private NpcVehicleSimulator npcVehicleSimulator;
        private NpcVehicleSpawner npcVehicleSpawner;
        private int currentSpawnNumber = 0;
        private int spawnPriority = 0;
        private TrafficSimNpcVehicle nextPrefabToSpawn = null;

        public void IncreasePriority(int priority)
        {
            spawnPriority += priority;
        }

        public void ResetPriority()
        {
            spawnPriority = 0;
        }

        public int GetCurrentPriority()
        {
            return spawnPriority;
        }

        public bool IsEnabled()
        {
            // TODO additional interface to disable route traffic
            return enabled && !IsMaximumSpawnsNumberReached();
        }

        public RouteTrafficSimulator(GameObject parent,
            TrafficSimNpcVehicle[] npcPrefabs,
            TrafficLane[] npcRoute,
            NpcVehicleSimulator vehicleSimulator,
            int maxSpawns = 0)
        {
            route = npcRoute;
            maximumSpawns = maxSpawns;
            npcVehicleSimulator = vehicleSimulator;
            TrafficLane[] spawnableLane = { route[0] };
            npcVehicleSpawner = new NpcVehicleSpawner(parent, npcPrefabs, spawnableLane);
        }

        public void GetRandomSpawnInfo(out NpcVehicleSpawnPoint spawnPoint, out TrafficSimNpcVehicle prefab)
        {
            // NPC prefab is randomly chosen and is fixed until it is spawned. This is due to avoid prefab "bound" race conditions
            // when smaller cars will always be chosen over larger ones while the spawning process checks if the given prefab can be 
            // put in the given position. 
            if (nextPrefabToSpawn == null)
            {
                nextPrefabToSpawn = npcVehicleSpawner.GetRandomPrefab();
            }
            prefab = nextPrefabToSpawn;

            // Spawn position is always chosen randomly from the set of all available lanes.
            // This avoids waiting for lanes which are already occupied, but there is another one available somewhere else.
            spawnPoint = npcVehicleSpawner.GetRandomSpawnPoint();
        }

        public bool Spawn(TrafficSimNpcVehicle prefab, NpcVehicleSpawnPoint spawnPoint, out TrafficSimNpcVehicle spawnedVehicle)
        {
            if (IsMaximumSpawnsNumberReached())
            {
                spawnedVehicle = null;
                return false;
            }
            ;

            if (npcVehicleSimulator.VehicleStates.Count >= npcVehicleSimulator.MaxVehicleCount)
            {
                spawnedVehicle = null;
                return false;
            }

            var vehicle = npcVehicleSpawner.Spawn(prefab, SpawnIdGenerator.Generate(), spawnPoint);
            npcVehicleSimulator.Register(vehicle, route.ToList<TrafficLane>(), spawnPoint.WaypointIndex);
            nextPrefabToSpawn = null;

            if (maximumSpawns > 0)
                currentSpawnNumber++;

            spawnedVehicle = vehicle;
            return true;
        }

        private bool IsMaximumSpawnsNumberReached()
        {
            return (currentSpawnNumber == maximumSpawns && maximumSpawns > 0);
        }
    }
}
