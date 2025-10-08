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
    /// Spawn point information of NPC vehicles.
    /// </summary>
    public struct NpcVehicleSpawnPoint
    {
        public TrafficLane Lane { get; }
        public int WaypointIndex { get; }
        public Vector3 Position { get; }
        public Vector3 Forward { get; }

        public NpcVehicleSpawnPoint(TrafficLane lane, int waypointIndex)
        {
            Lane = lane;
            WaypointIndex = waypointIndex;
            Position = lane.Waypoints[waypointIndex];
            Forward = waypointIndex == lane.Waypoints.Length - 1
                ? Position - lane.Waypoints[waypointIndex - 1]
                : lane.Waypoints[waypointIndex + 1] - Position;
        }
    }

    /// <summary>
    /// This class spawns and despawns NPC vehicles.
    /// </summary>
    public class NpcVehicleSpawner
    {
        /// <summary>
        /// Get random NPC vehicle prefab.
        /// </summary>
        /// <returns>NPC vehicle prefab</returns>
        public TrafficSimNpcVehicle GetRandomPrefab()
            => _prefabs[Random.Range(0, _prefabs.Length)];

        /// <summary>
        /// Get random spawn point.
        /// </summary>
        /// <returns>Spawn point</returns>
        public NpcVehicleSpawnPoint GetRandomSpawnPoint()
            => _spawnPoints[Random.Range(0, _spawnPoints.Length)];

        TrafficSimNpcVehicle[] _prefabs;
        NpcVehicleSpawnPoint[] _spawnPoints;
        GameObject _NpcVehicleParentsObj;

        /// <summary>
        /// Initialize <see cref="NpcVehicleSpawner"/>.
        /// </summary>
        /// <param name="prefabs">NPC vehicle prefabs to be spawned.</param>
        /// <param name="spawnableLanes">Lanes where vehicles can spawn.</param>
        public NpcVehicleSpawner(GameObject parentsObj, TrafficSimNpcVehicle[] prefabs, TrafficLane[] spawnableLanes)
        {
            this._NpcVehicleParentsObj = parentsObj;
            this._prefabs = prefabs;
            this._spawnPoints = new NpcVehicleSpawnPoint[spawnableLanes.Length];
            for (var i = 0; i < spawnableLanes.Length; i++)
            {
                this._spawnPoints[i] = new NpcVehicleSpawnPoint(spawnableLanes[i], 0);
            }
        }

        /// <summary>
        /// Check if the vehicle with <paramref name="localBounds"/> can be spawned at <paramref name="npcVehicleSpawnPoint"/>.
        /// </summary>
        public static bool IsSpawnable(Bounds localBounds, NpcVehicleSpawnPoint npcVehicleSpawnPoint)
        {
            var rotation = Quaternion.LookRotation(npcVehicleSpawnPoint.Forward);
            var center = rotation * localBounds.center + npcVehicleSpawnPoint.Position;

            // Raise the center 1 m above the ground.
            center += new Vector3(0, 1f, 0);

            var result = Physics.CheckBox(
                center,
                localBounds.extents,
                rotation);

            return !result;
        }

        /// <summary>
        /// Get a <see cref="NpcVehicleSpawnPoint"/> from spawnable lanes.
        /// Return false if vehicles are present in all lanes.
        /// </summary>
        /// <param name="prefab">Prefab to be spawned. Bounds of the prefab is used for collision checking.</param>
        /// <param name="npcVehicleSpawnPoint">A point that vehicles can spawn.</param>
        /// <returns>Whether there was a spawnable point.</returns>
        public bool TryGetRandomSpawnablePoint(GameObject prefab, out NpcVehicleSpawnPoint npcVehicleSpawnPoint)
        {
            var bounds = prefab.GetComponent<TrafficSimNpcVehicle>().NpcVehicle.Bounds;
            var spawnablePoints = GetSpawnablePoints(bounds);
            if (spawnablePoints.Count == 0)
            {
                npcVehicleSpawnPoint = default;
                return false;
            }
            npcVehicleSpawnPoint = spawnablePoints[Random.Range(0, spawnablePoints.Count)];
            return true;
        }

        /// <summary>
        /// Spawn NPC vehicle prefab.
        /// Use <see cref="TryGetRandomSpawnablePoint"/> to find spawn point.
        /// </summary>
        /// <param name="prefab">NPC vehicle prefab</param>
        /// <param name="npcVehicleSpawnPoint">Spawn point</param>
        /// <returns>Spawned NPC vehicle.</returns>
        public TrafficSimNpcVehicle Spawn(TrafficSimNpcVehicle prefab, uint vehicleId, NpcVehicleSpawnPoint npcVehicleSpawnPoint)
        {
            var obj = Object.Instantiate(prefab, npcVehicleSpawnPoint.Position, Quaternion.identity);
            obj.name = obj.name + "_" + vehicleId.ToString();
            obj.transform.forward = npcVehicleSpawnPoint.Forward;
            obj.transform.parent = _NpcVehicleParentsObj.transform;
            var vehicle = obj.GetComponent<TrafficSimNpcVehicle>();
            vehicle.Initialize(vehicleId);

            return vehicle;
        }

        /// <summary>
        /// Destroy <paramref name="vehicle"/>
        /// </summary>
        public void Despawn(TrafficSimNpcVehicle vehicle)
        {
            Object.DestroyImmediate(vehicle.gameObject);
        }


        /// <summary>
        /// Get all spawnable point from spawnable lanes.
        /// </summary>
        public IList<NpcVehicleSpawnPoint> GetSpawnablePoints(Bounds localBounds)
        {
            var spawnablePoints = new List<NpcVehicleSpawnPoint>();
            foreach (var spawnPoint in _spawnPoints)
            {
                if (IsSpawnable(localBounds, spawnPoint))
                    spawnablePoints.Add(spawnPoint);
            }

            return spawnablePoints;
        }
    }
}
