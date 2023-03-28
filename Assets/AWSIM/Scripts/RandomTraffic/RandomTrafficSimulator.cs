using UnityEngine;
using Random = UnityEngine.Random;

namespace AWSIM.RandomTraffic
{
    /// <summary>
    /// Randomly simulate NPC traffic. (Currently only Vehicle is supported)
    /// - Automatic generation of NPC routes from VectorMap
    /// - Random NPC traffic
    /// - Traffic light control based on the Vienna Convention
    /// - Reproducibility by Seed value
    /// </summary>
    public class RandomTrafficSimulator : MonoBehaviour
    {
        [SerializeField, Tooltip("Seed value for random generator.")]
        private int seed;
        [SerializeField] private LayerMask vehicleLayerMask;
        [SerializeField] private LayerMask groundLayerMask;

        [Header("NPC Vehicle Settings")]
        [SerializeField] private int maxVehicleCount = 40;
        [SerializeField] private GameObject[] npcPrefabs;
        [SerializeField, Tooltip("TrafficLanes where NPC vehicles can spawn.")]
        private TrafficLane[] spawnableLanes;
        [SerializeField] private NPCVehicleConfig vehicleConfig;

        [Header("Debug")]
        [SerializeField] private bool showGizmos = false;

        private uint numberSpawnedVehicles = 0;
        private NPCVehicleSpawner npcVehicleSpawner;
        private NPCVehicleSimulator npcVehicleSimulator;

        private void Awake()
        {
            Random.InitState(seed);

            npcVehicleSimulator = new NPCVehicleSimulator(vehicleConfig, vehicleLayerMask, groundLayerMask, maxVehicleCount);

            npcVehicleSpawner = new NPCVehicleSpawner(this.gameObject, npcPrefabs, spawnableLanes);
        }

        private void FixedUpdate()
        {
            SpawnRandom();

            npcVehicleSimulator.Update(Time.fixedDeltaTime);

            Despawn();
        }

        private void Reset()
        {
            vehicleConfig = NPCVehicleConfig.Default();

            vehicleLayerMask = LayerMask.GetMask(Constants.Layers.Vehicle);
            groundLayerMask = LayerMask.GetMask(Constants.Layers.Ground);
        }

        private void SpawnRandom()
        {
            if (npcVehicleSimulator.VehicleStates.Count >= maxVehicleCount)
                return;

            var prefab = npcVehicleSpawner.GetRandomPrefab();
            if (!npcVehicleSpawner.TryGetRandomSpawnablePoint(prefab, out var spawnPoint))
                return;

            var vehicle = npcVehicleSpawner.Spawn(prefab, numberSpawnedVehicles++, spawnPoint);
            npcVehicleSimulator.Register(vehicle, spawnPoint.Lane, spawnPoint.WaypointIndex);
        }

        private void Despawn()
        {
            foreach (var state in npcVehicleSimulator.VehicleStates)
            {
                if (state.ShouldDespawn)
                    npcVehicleSpawner.Despawn(state.Vehicle);
            }
            npcVehicleSimulator.RemoveInvalidVehicles();
        }

        private void OnDestroy()
        {
            npcVehicleSimulator?.Dispose();
        }

        private void OnDrawGizmos()
        {
            if (!showGizmos)
                return;

            var defaultColor = Gizmos.color;

            npcVehicleSimulator?.ShowGizmos();

            Gizmos.color = defaultColor;
        }
    }
}
