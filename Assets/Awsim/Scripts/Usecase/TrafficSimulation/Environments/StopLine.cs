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
using Awsim.Entity;

namespace Awsim.Usecase.TrafficSimulation
{
    /// <summary>
    /// Stop line component.
    /// </summary>
    public class StopLine : MonoBehaviour
    {

        /// <summary>
        /// Get line data consists of 2 points.
        /// </summary>
        public Vector3[] Points => _points;

        /// <summary>
        /// Get center point of the stop line.
        /// </summary>
        public Vector3 CenterPoint => (_points[0] + _points[1]) / 2f;

        [SerializeField, Tooltip("Line data consists of 2 points.")]
        Vector3[] _points = new Vector3[2];

        [SerializeField, Tooltip("Indicates whether the stop sign exists.")]
        bool _hasStopSign = false;

        [SerializeField, Tooltip("Traffic light ")]
        TrafficLight _trafficLight;

        public TrafficLight TrafficLight
        {
            get => _trafficLight;
            set => _trafficLight = value;
        }

        public bool HasStopSign
        {
            get => _hasStopSign;
            set => _hasStopSign = value;
        }

        public static StopLine Create(Vector3 p1, Vector3 p2)
        {
            var gameObject = new GameObject("StopLine", typeof(StopLine));
            gameObject.transform.position = p1;
            var stopLine = gameObject.GetComponent<StopLine>();
            stopLine._points[0] = p1;
            stopLine._points[1] = p2;
            return stopLine;
        }
    }
}
