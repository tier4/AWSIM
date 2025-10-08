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
using System.Linq;
using UnityEngine;
using Awsim.Common;

namespace Awsim.Usecase.TrafficSimulation
{
    /// <summary>
    /// Component for managing traffic simulators. Traffic manager collets all traffic simulators and manages the spawning process.
    /// - Reproducibility by Seed value
    /// </summary>
    public class TrafficSimulator : MonoBehaviour
    {
        public int Seed => _seed;
        public int MaxVehicleCount => _maxVehicleCount;

        [SerializeField, Tooltip("Ego vehicle handler. If not set, the manager creates a dummy ego. This reference is also set automatically when the Ego spawns via the traffic simulator.")]
        Transform _egoVehicle;

        [SerializeField, Tooltip("Seed value for random generator.")]
        int _seed;

        [SerializeField]
        TrafficIntersection[] _trafficIntersections;


        [Header("NPC Vehicle Settings")]
        [SerializeField] NpcVehicleConfig _vehicleConfig = NpcVehicleConfig.Default();

        [SerializeField, Tooltip("Obstacle layer for raytracing the collision distances.")]
        LayerMask _obstacleLayerMask;

        [SerializeField, Tooltip("Ground layer for raytracing the collision distances.")]
        LayerMask _groundLayerMask;

        [SerializeField, Tooltip("A maximum number of vehicles that can simultaneously live in the scene. Lowering this value results in less dense traffic but improves the simulator's performance.")]
        int _maxVehicleCount = 40;

        [SerializeField, Tooltip("A minimal distance between the EGO and the NPC to spawn")]
        float _spawnDistanceToEgo = 50.0f;

        [Header("Debug")]
        [SerializeField] protected bool _showGizmos = false;
        [SerializeField] protected bool _showYieldingPhase = false;
        [SerializeField] protected bool _showObstacleChecking = false;
        [SerializeField] protected bool _showSpawnPoints = false;
        [SerializeField] RandomTrafficSimulatorConfiguration[] _randomTrafficSims;
        [SerializeField] RouteTrafficSimulatorConfiguration[] _routeTrafficSims;
        [SerializeField] NpcVehicleSimulator _npcVehicleSimulator;
        List<ITrafficSimulator> _trafficSimulatorNodes;
        Dictionary<NpcVehicleSpawnPoint, Dictionary<ITrafficSimulator, TrafficSimNpcVehicle>> _spawnLanes;
        List<TrafficSimNpcVehicle> _trafficSimNpcVehicleList;

        bool _isTrafficIntersectionInitialized = false;     // TODO: Consider how to reset the traffic simulator without having flags.

        /// <summary>
        /// Adds traffic simulator to the manager.
        /// </summary>
        public void AddTrafficSimulator(ITrafficSimulator simulator)
        {
            _trafficSimulatorNodes.Add(simulator);
        }

        /// <summary>
        /// Clears all NPC vehicles running in simulation.
        /// </summary>
        public void ClearAll()
        {
            _trafficSimulatorNodes.Clear();
            _npcVehicleSimulator?.ClearAll();
        }

        public void Initialize()
        {
            _trafficSimNpcVehicleList = new List<TrafficSimNpcVehicle>();

            if (!_isTrafficIntersectionInitialized)
            {
                foreach (var e in _trafficIntersections)
                    e.Initialize();
                _isTrafficIntersectionInitialized = true;
            }

            Random.InitState(_seed);

            _spawnLanes = new Dictionary<NpcVehicleSpawnPoint, Dictionary<ITrafficSimulator, TrafficSimNpcVehicle>>();

            _npcVehicleSimulator = new NpcVehicleSimulator(_vehicleConfig, _obstacleLayerMask, _groundLayerMask, _maxVehicleCount, _egoVehicle);

            verifyIntegrationEnvironmentElements();
            _trafficSimulatorNodes = new List<ITrafficSimulator>();
            if (_npcVehicleSimulator == null)
            {
                Debug.LogError("Traffic manager requires NPC Vehicle Simulator script.");
                return;
            }

            foreach (var randomTrafficConf in _randomTrafficSims)
            {
                RandomTrafficSimulator randomTs = new RandomTrafficSimulator(
                    this.gameObject,
                    randomTrafficConf.TrafficSimNpcVehiclePrefabs,
                    randomTrafficConf.SpawnableTrafficLanes,
                    _npcVehicleSimulator,
                    randomTrafficConf.SpawnCountLimit
                );
                randomTs.enabled = randomTrafficConf.EnableSimulation;
                _trafficSimulatorNodes.Add(randomTs);
            }

            foreach (var routeTrafficSimConf in _routeTrafficSims)
            {
                RouteTrafficSimulator routeTs = new RouteTrafficSimulator(
                    this.gameObject,
                    routeTrafficSimConf.TrafficSimNpcVehiclePrefabs,
                    routeTrafficSimConf.RouteLanes,
                    _npcVehicleSimulator,
                    routeTrafficSimConf.SpawnCountLimit
                );
                routeTs.enabled = routeTrafficSimConf.EnableSimulation;
                _trafficSimulatorNodes.Add(routeTs);
            }
        }

        public void Initialize(int seed, int maxVehicleCount)
        {
            _seed = seed;
            _maxVehicleCount = maxVehicleCount;

            Initialize();
        }

        public void OnFixedUpdate()
        {
            foreach (var e in _trafficSimNpcVehicleList)
                e.OnFixedUpdate();

            // Manage NPC spawning with the traffic simulators

            // Clear null elements in current list of traffic simulator
            _trafficSimulatorNodes.RemoveAll(item => item == null);

            // Find out which lanes are used by multiple spawners
            // We can easly spawn vehicle on a lane which only one spawner hooked to that lane.
            // If there are multiple spawners for one lane, we need to manage spawning. Without
            // managing, one spawner might overcome all others because of the vehicle size check.
            foreach (var trafficSimulator in _trafficSimulatorNodes)
            {
                if (!trafficSimulator.IsEnabled()) continue;

                trafficSimulator.GetRandomSpawnInfo(out var spawnPoint, out var prefab);
                if (_spawnLanes.ContainsKey(spawnPoint))
                {
                    _spawnLanes[spawnPoint].Add(trafficSimulator, prefab);
                }
                else
                {
                    var tsims = new Dictionary<ITrafficSimulator, TrafficSimNpcVehicle>();
                    tsims.Add(trafficSimulator, prefab);
                    _spawnLanes.Add(spawnPoint, tsims);
                }
            }

            // For lane with single vehicle spawner - just spawn it.
            // For lane with multiple vehicle spawners - make priorities and spawn one by one.
            foreach (var spawnLoc in _spawnLanes)
            {
                TrafficSimNpcVehicle spawnedVehicle;

                var distance2D = LaneletGeometryUtility.Distance2D(_egoVehicle.transform.position, spawnLoc.Key.Position);
                if (distance2D < _spawnDistanceToEgo)
                {
                    continue;
                }

                if (spawnLoc.Value.Count == 1)
                {
                    var tsimAndPrefab = spawnLoc.Value.First();
                    var trafficSim = tsimAndPrefab.Key;
                    var prefab = tsimAndPrefab.Value;
                    if (!NpcVehicleSpawner.IsSpawnable(prefab.NpcVehicle.Bounds, spawnLoc.Key))
                        continue;
                    var spawned = trafficSim.Spawn(prefab, spawnLoc.Key, out spawnedVehicle);
                }
                else
                {
                    var priorityTrafficSimList = spawnLoc.Value.OrderByDescending(x => x.Key.GetCurrentPriority());
                    var priorityTrafficSimGo = priorityTrafficSimList.First();
                    var prefab = priorityTrafficSimGo.Value;
                    if (!NpcVehicleSpawner.IsSpawnable(prefab.NpcVehicle.Bounds, spawnLoc.Key))
                    {
                        continue;
                    }
                    bool spawned = priorityTrafficSimGo.Key.Spawn(prefab, spawnLoc.Key, out spawnedVehicle);
                    if (spawned)
                    {
                        foreach (var rest in priorityTrafficSimList)
                        {
                            rest.Key.IncreasePriority(1);
                        }
                        priorityTrafficSimGo.Key.ResetPriority();
                    }
                }

                if (spawnedVehicle != null)
                    _trafficSimNpcVehicleList.Add(spawnedVehicle);
            }

            _spawnLanes.Clear();
            _npcVehicleSimulator.StepOnce(Time.fixedDeltaTime);

            Despawn();
        }

        public void OnUpdate()
        {
            foreach (var e in _trafficIntersections)
                e.OnUpdate();

            foreach (var e in _trafficSimNpcVehicleList)
                e.OnUpdate();
        }

        public void Restart(int newSeed = 0, int newMaxVehicleCount = 10)
        {
            Dispose();

            _seed = newSeed;
            _maxVehicleCount = newMaxVehicleCount;

            Initialize();
        }

        void verifyIntegrationEnvironmentElements()
        {
            GameObject trafficLanesObject = GameObject.Find("TrafficLanes");
            if (trafficLanesObject == null)
            {
                Debug.LogError("VerifyIntegrationEnvironmentElements error: Object 'TrafficLanes' not found in the scene.");
            }

            Transform[] children = trafficLanesObject.GetComponentsInChildren<Transform>();
            HashSet<string> uniqueNames = new HashSet<string>();
            bool isAnyIntersectionLane = false;
            bool isAnyTrafficScript = false;
            foreach (Transform child in children)
            {
                var trafficScript = child.gameObject.GetComponent<TrafficLane>();
                if (trafficScript)
                {
                    isAnyTrafficScript = true;
                    if (trafficScript._intersectionLane)
                    {
                        isAnyIntersectionLane = true;
                    }
                    if (!uniqueNames.Add(child.name))
                    {
                        Debug.LogError("VerifyIntegrationEnvironmentElements error: Found repeated child name in the 'TrafficLanes' object: " + child.name);
                    }
                }
            }
            if (!isAnyIntersectionLane)
            {
                Debug.LogError("VerifyIntegrationEnvironmentElements error: Not found any TrafficLane with 'IntersectionLane' set to true.");
            }
            if (!isAnyTrafficScript)
            {
                Debug.LogError("VerifyIntegrationEnvironmentElements error: Not found any TrafficLane with 'TrafficScript'.");
            }

        }

        void Despawn()
        {
            foreach (var state in _npcVehicleSimulator.VehicleStates)
            {
                if (state.ShouldDespawn)
                {
                    Object.DestroyImmediate(state.Vehicle.gameObject);
                    _trafficSimNpcVehicleList.Remove(state.Vehicle);
                }
            }
            _npcVehicleSimulator.RemoveInvalidVehicles();
        }

        void OnDestroy()
        {
            _npcVehicleSimulator?.Dispose();
        }

        void DrawSpawnPoints()
        {
            Gizmos.color = Color.cyan;

            foreach (var randomTrafficConf in _randomTrafficSims)
            {
                foreach (var lane in randomTrafficConf.SpawnableTrafficLanes)
                {
                    if (lane != null)
                        Gizmos.DrawCube(lane.Waypoints[0], new Vector3(2.5f, 0.2f, 2.5f));
                }
            }

            Gizmos.color = Color.magenta;

            foreach (var routeTrafficSimConf in _routeTrafficSims)
            {
                if (routeTrafficSimConf.RouteLanes[0] != null)
                    Gizmos.DrawCube(routeTrafficSimConf.RouteLanes[0].Waypoints[0], new Vector3(2.5f, 0.2f, 2.5f));

            }
        }

        void OnDrawGizmos()
        {
            if (!_showGizmos)
                return;

            var defaultColor = Gizmos.color;
            _npcVehicleSimulator?.ShowGizmos(_showYieldingPhase, _showObstacleChecking);
            if (_showSpawnPoints)
                DrawSpawnPoints();

            Gizmos.color = defaultColor;
        }

        void Dispose()
        {
            _npcVehicleSimulator?.Dispose();
            ClearAll();
            Despawn();
            _spawnLanes.Clear();
        }
    }
}
