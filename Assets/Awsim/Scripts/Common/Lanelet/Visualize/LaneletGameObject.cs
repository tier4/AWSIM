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

// TODO: think of a better place to put this
// TODO: think of a better name

namespace Awsim.Common
{
    public class LaneletGameObject : MonoBehaviour
    {
        [SerializeField, Tooltip("Waypoints in this lane.")]
        Vector3[] _waypoints;

        public Vector3[] Waypoints => _waypoints;

        /// <summary>
        /// Create <see cref="LaneletGameObject"/> instance in the scene.
        /// </summary>
        /// <param name="wayPoints"></param>
        /// <returns><see cref="LaneletGameObject"/> instance.</returns>
        public static LaneletGameObject Create(Vector3[] wayPoints)
        {
            var gameObject = new GameObject("LaneletBound", typeof(LaneletGameObject));
            gameObject.transform.position = wayPoints[0];
            var laneletBound = gameObject.GetComponent<LaneletGameObject>();
            laneletBound._waypoints = wayPoints;
            return laneletBound;
        }
    }
}
