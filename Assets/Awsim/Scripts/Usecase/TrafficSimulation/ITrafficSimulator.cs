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

namespace Awsim.Usecase.TrafficSimulation
{

    /// <summary>
    /// Interface for traffic simulators
    /// </summary>  
    public interface ITrafficSimulator
    {
        /// <summary>
        /// Try to spawn a prefab in a given spawn location. 
        /// </summary>
        /// <param name="prefab"></param>
        /// <param name="spawnPoint"></param>
        /// <param name="spawnedVehicle"></param>
        /// <returns>True if spawned, False is failed to spawn.</returns>
        public bool Spawn(TrafficSimNpcVehicle prefab, NpcVehicleSpawnPoint spawnPoint, out TrafficSimNpcVehicle spawnedVehicle);

        /// <summary>
        /// Gets random spawn point and NPC prefab.
        /// </summary>
        public void GetRandomSpawnInfo(out NpcVehicleSpawnPoint spawnPoint, out TrafficSimNpcVehicle prefab);

        /// <summary>
        /// Increase the traffic simulator spawn priority.
        /// <param name="spawnPoint"></param>
        /// </summary>
        public void IncreasePriority(int priority);

        /// <summary>
        /// Get current traffic simulator spawn priority.
        /// <returns>Priority value</returns>
        /// </summary>
        public int GetCurrentPriority();

        /// <summary>
        /// Reset traffic simulator spawn priority.
        /// </summary>
        public void ResetPriority();

        /// <summary>
        /// Is the traffic simulator enabled.
        /// </summary>
        public bool IsEnabled();
    }
}
