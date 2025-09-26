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
using System;

namespace Awsim.Usecase.TrafficSimulation
{
    [Serializable]
    public class RandomTrafficSimulatorConfiguration
    {
        public bool EnableSimulation => _enableSimulation;

        public TrafficSimNpcVehicle[] TrafficSimNpcVehiclePrefabs => _trafficSimNpcVehiclePrefabs;

        public TrafficLane[] SpawnableTrafficLanes => _spawnableTrafficLanes;

        public bool EnableSpawnCountLimit => _enableSpawnCountLimit;

        public int SpawnCountLimit => _spawnCountLimit;

        [Tooltip("Is this traffic simulation enabled.")]
        [SerializeField]
        bool _enableSimulation;

        [Tooltip("NPCs to be spawned.")]
        [SerializeField]
        TrafficSimNpcVehicle[] _trafficSimNpcVehiclePrefabs;

        [Tooltip("TrafficLanes where NPC vehicles can spawn.")]
        [SerializeField]
        TrafficLane[] _spawnableTrafficLanes;

        [SerializeField]
        bool _enableSpawnCountLimit;

        [Tooltip("Describes the lifetime of a traffic simulator instance by specifying how many vehicles this traffic simulator will spawn. Setting it makes the spawner live longer or shorter, while it can also be set to infinity if needed (endless lifetime).")]
        [SerializeField]
        int _spawnCountLimit;
    }

    /// <summary>
    /// Randomly simulate NPC traffic. (Currently only Vehicle is supported)
    /// - Automatic generation of NPC routes from VectorMap
    /// - Random NPC traffic
    /// - Traffic light control based on the Vienna Convention
    /// - Reproducibility by Seed value
    /// </summary>
    public class RandomTrafficSimulator : ITrafficSimulator
    {
        [SerializeField, Tooltip("Is the traffic enabled")]
        public bool enabled = true;

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

        public int GetCurrentPriority()
        {
            return spawnPriority;
        }

        public void ResetPriority()
        {
            spawnPriority = 0;
        }

        public bool IsEnabled()
        {
            // TODO an interface to disable random traffic
            return enabled && !IsMaximumSpawnsNumberReached();
        }

        public RandomTrafficSimulator(GameObject parent,
            TrafficSimNpcVehicle[]
            prefabs,
            TrafficLane[]
            spawnableLanes,
            NpcVehicleSimulator
            vehicleSimulator,
            int maxSpawns = 0)
        {
            maximumSpawns = maxSpawns;
            npcVehicleSimulator = vehicleSimulator;
            npcVehicleSpawner = new NpcVehicleSpawner(parent, prefabs, spawnableLanes);
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
            npcVehicleSimulator.Register(vehicle, spawnPoint.Lane, spawnPoint.WaypointIndex);
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
