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
using UnityEngine.Serialization;

namespace Awsim.Usecase.TrafficSimulation
{
    /// <summary>
    /// Traffic lane component.
    /// </summary>
    public class TrafficLane : MonoBehaviour
    {
        /// <summary>
        /// Turning direction type of vehicles.
        /// </summary>
        public enum TurnDirectionType
        {
            Straight = 0,
            Left = 1,
            Right = 2,
            Null = 3
        }

        /// <summary>
        /// Get a stop line in the lane.
        /// </summary>
        public StopLine StopLine
        {
            get => _stopLine;
            set => _stopLine = value;
        }

        public Vector3 GetStopPoint(int waypointIndex = 0)
        {
            return StopLine == null ? Waypoints[waypointIndex] : StopLine.CenterPoint;
        }

        [FormerlySerializedAs("intersectionLane")]
        [SerializeField, Tooltip("Is intersection lane")]
        public bool _intersectionLane;

        /// <summary>
        /// Get waypoints in this lane.
        /// </summary>
        public Vector3[] Waypoints => _waypoints;

        /// <summary>
        /// Get turning direction of vehicles in the lane.
        /// </summary>
        public TurnDirectionType TurnDirection => _turnDirection;

        /// <summary>
        /// Get next lanes connected to this lane.
        /// </summary>
        public List<TrafficLane> NextLanes => _nextLanes;

        /// <summary>
        /// Get lanes leading to this lane.
        /// </summary>
        public List<TrafficLane> PrevLanes => _prevLanes;

        /// <summary>
        /// Get lanes to which vehicles in this lane should yield the right of way.
        /// </summary>
        public List<TrafficLane> RightOfWayLanes => _rightOfWayLanes;

        /// <summary>
        /// Get speed limit in m/s.
        /// </summary>
        public float SpeedLimit => _speedLimit;

        [FormerlySerializedAs("waypoints")]
        [SerializeField, Tooltip("Waypoints in this lane.")]
        Vector3[] _waypoints;

        [FormerlySerializedAs("turnDirection")]
        [SerializeField, Tooltip("Turning direction of vehicles in the lane.")]
        TurnDirectionType _turnDirection;

        [FormerlySerializedAs("nextLanes")]
        [SerializeField, Tooltip("Next lanes connected to this lane.")]
        List<TrafficLane> _nextLanes = new List<TrafficLane>();

        [FormerlySerializedAs("prevLanes")]
        [SerializeField, Tooltip("Lanes leading to this lane.")]
        List<TrafficLane> _prevLanes = new List<TrafficLane>();

        [FormerlySerializedAs("rightOfWayLanes")]
        [SerializeField, Tooltip("Lanes to which vehicles in this lane should yield the right of way.")]
        List<TrafficLane> _rightOfWayLanes = new List<TrafficLane>();

        [FormerlySerializedAs("stopLine")]
        [SerializeField, Tooltip("Stop line in the lane")]
        StopLine _stopLine;

        [FormerlySerializedAs("speedLimit")]
        [SerializeField, Tooltip("Speed limit in m/s")]
        float _speedLimit;

        /// <summary>
        /// Create <see cref="TrafficLane"/> instance in the scene.<br/>
        /// </summary>
        /// <param name="wayPoints"></param>
        /// <param name="speedLimit"></param>
        /// <returns><see cref="TrafficLane"/> instance.</returns>
        public static TrafficLane Create(Vector3[] wayPoints, TurnDirectionType turnDirection, float speedLimit = 0f)
        {
            var gameObject = new GameObject("TrafficLane", typeof(TrafficLane));
            gameObject.transform.position = wayPoints[0];
            var trafficLane = gameObject.GetComponent<TrafficLane>();
            trafficLane._waypoints = wayPoints;
            trafficLane._turnDirection = turnDirection;
            trafficLane._speedLimit = speedLimit;
            return trafficLane;
        }
    }
}
