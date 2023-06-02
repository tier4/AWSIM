using UnityEngine;
using System;
using System.Linq;

namespace AWSIM.TrafficSimulation
{
    [Serializable]
    public struct RouteTrafficSimulatorConfiguration {
        /// <summary>
        /// Available NPC prefabs
        /// </summary>
        [Tooltip("NPCs to be spawned.")]
        public GameObject[] npcPrefabs;

        [Tooltip("Route to follow. The first element also acts as a spawn lane.")]
        public TrafficLane[] route;

        [Tooltip("Describes the lifetime of a traffic simulator instance by specifying how many vehicles this traffic simulator will spawn. Setting it makes the spawner live longer or shorter, while it can also be set to infinity if needed (endless lifetime).")]
        public int maximumSpawns;

        [Tooltip("Is this traffic simulation enabled.")]
        public bool enabled;
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
        private NPCVehicleSimulator npcVehicleSimulator;
        private NPCVehicleSpawner npcVehicleSpawner;
        private int currentSpawnNumber = 0;
        private int spawnPriority = 0;
        private GameObject nextPrefabToSpawn = null;

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
            GameObject[] npcPrefabs,
            TrafficLane[] npcRoute,
            NPCVehicleSimulator vehicleSimulator,
            int maxSpawns = 0)
        {
            route = npcRoute;
            maximumSpawns = maxSpawns;
            npcVehicleSimulator = vehicleSimulator;
            TrafficLane[] spawnableLane = {route[0]};
            npcVehicleSpawner = new NPCVehicleSpawner(parent, npcPrefabs, spawnableLane); 
        }

        public void GetRandomSpawnInfo(out NPCVehicleSpawnPoint spawnPoint, out GameObject prefab)
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

        public bool Spawn(GameObject prefab, NPCVehicleSpawnPoint spawnPoint, out NPCVehicle spawnedVehicle)
        {
            if(IsMaximumSpawnsNumberReached()) {
                spawnedVehicle = null; 
                return false;
            };

            if (npcVehicleSimulator.VehicleStates.Count >= npcVehicleSimulator.maxVehicleCount)
            {
                spawnedVehicle = null; 
                return false;
            }

            var vehicle = npcVehicleSpawner.Spawn(prefab, SpawnIdGenerator.Generate(), spawnPoint);
            npcVehicleSimulator.Register(vehicle, route.ToList<TrafficLane>(), spawnPoint.WaypointIndex);
            nextPrefabToSpawn = null;
            
            if(maximumSpawns > 0)
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
