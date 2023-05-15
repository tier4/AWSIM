// Copyright 2023 Tier4.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using AWSIM.Lanelet;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;
using GeometryUtility = AWSIM.Lanelet.GeometryUtility;

namespace AWSIM.RandomTraffic
{
    /// <summary>
    /// A class that provides capability to convert Lanelet2 data into environment components.
    /// </summary>
    public class LaneletBoundsLoader
    {
        [System.Serializable]
        public struct WaypointSettings
        {
            [SerializeField, Range(0.1f, 2f)]
            [Tooltip("Resolution of resampling. Lower values provide better accuracy at the cost of processing time.")]
            private float resolution;
            [SerializeField, Range(5f, 100f)]
            [Tooltip("Minimum length(m) between adjacent points.")]
            private float minDeltaLength;
            [SerializeField, Range(5f, 30f)]
            [Tooltip("Minimum angle(deg) between adjacent edges.")]
            private float minDeltaAngle;

            /// <summary>
            /// Resolution of resampling. Lower values provide better accuracy at the cost of processing time.
            /// </summary>
            public float Resolution => resolution;

            /// <summary>
            /// Minimum length(m) between adjacent points.
            /// </summary>
            public float MinDeltaLength => minDeltaLength;

            /// <summary>
            /// Minimum angle(deg) between adjacent edges.
            /// </summary>
            public float MinDeltaAngle => minDeltaAngle;

            public WaypointSettings(float resolution, float minDeltaLength, float minDeltaAngle)
            {
                this.resolution = resolution;
                this.minDeltaLength = minDeltaLength;
                this.minDeltaAngle = minDeltaAngle;
            }

            public static WaypointSettings Default()
            {
                return new WaypointSettings(1f, 30f, 15f);
            }
        }

        // Internal variables for mapping lanelet id to components
        private readonly Dictionary<long, TrafficLane> trafficLanesLeft = new Dictionary<long, TrafficLane>();
        private readonly Dictionary<long, TrafficLane> trafficLanesRight = new Dictionary<long, TrafficLane>();

        private LaneletMap laneletMap;
        private GameObject trafficLaneBoundHolder;
        private WaypointSettings settings = WaypointSettings.Default();

        public void SetWaypointSettings(WaypointSettings settings)
        {
            this.settings = settings;
        }

        /// <summary>
        /// Load <see cref="OsmData"/> and generate environment components of AWSIM.
        /// </summary>
        /// <param name="osmData">Input OSM data.</param>
        /// <param name="offset">Offset from lanelet origin to Unity scene origin. Usually this is a MGRS coordinate of Unity scene origin.</param>
        /// <param name="holder">GameObject to that generated objects are placed.</param>
        public void Load(OsmData osmData, Vector3 offset, GameObject holder)
        {
            laneletMap = new OsmToLaneletMap(offset).Convert(osmData);
            trafficLaneBoundHolder = new GameObject("TrafficLanesBound");
            if (holder != null)
            {
                trafficLaneBoundHolder.transform.parent = holder.transform;
            }

            CreateTrafficLanes();
            SetTrafficLaneConnections();
            SetRightOfWays();
        }

        private void CreateTrafficLanes()
        {
            foreach (var lanelet in laneletMap.Lanelets.Values)
            {
                if (lanelet.Attributes[AttributeKeys.Subtype] != AttributeValues.Road)
                {
                    continue;
                }

                lanelet.Attributes.TryGetValue(AttributeKeys.SpeedLimit, out float speedLimitKph);
                // Convert km/h -> m/s
                var speedLimitMps = speedLimitKph / 3600f * 1000f;
                var waypointsLeft = lanelet.CalculateLeftLine(settings.Resolution, settings.MinDeltaLength, settings.MinDeltaAngle);
                var waypointsRight = lanelet.CalculateRightLine(settings.Resolution, settings.MinDeltaLength, settings.MinDeltaAngle);
                lanelet.Attributes.TryGetValue(AttributeKeys.TurnDirection, out string turnDirectionString);
                var turnDirection =
                    turnDirectionString == AttributeValues.Straight ? TrafficLane.TurnDirectionType.STRAIGHT :
                    turnDirectionString == AttributeValues.Left ? TrafficLane.TurnDirectionType.LEFT :
                    turnDirectionString == AttributeValues.Right ? TrafficLane.TurnDirectionType.RIGHT
                    : TrafficLane.TurnDirectionType.STRAIGHT;
                var trafficLaneLeft = TrafficLane.Create(waypointsLeft, turnDirection, speedLimitMps);
                trafficLaneLeft.transform.parent = trafficLaneBoundHolder.transform;
                trafficLanesLeft.Add(lanelet.ID, trafficLaneLeft);
                var trafficLaneRight = TrafficLane.Create(waypointsRight, turnDirection, speedLimitMps);
                trafficLaneRight.transform.parent = trafficLaneBoundHolder.transform;
                trafficLanesRight.Add(lanelet.ID, trafficLaneRight);
            }
        }

        private void SetTrafficLaneConnections()
        {
            var connections = FindLaneletConnections(laneletMap);
            foreach (var entry in trafficLanesLeft)
            {
                var id = entry.Key;
                var trafficLane = entry.Value;
                foreach (var (id1, id2) in connections)
                {
                    if (id == id1 && trafficLanesLeft.ContainsKey(id2))
                    {
                        trafficLane.NextLanes.Add(trafficLanesLeft[id2]);
                        trafficLanesLeft[id2].PrevLanes.Add(trafficLane);
                    }
                }
            }
            foreach (var entry in trafficLanesRight)
            {
                var id = entry.Key;
                var trafficLane = entry.Value;
                foreach (var (id1, id2) in connections)
                {
                    if (id == id1 && trafficLanesRight.ContainsKey(id2))
                    {
                        trafficLane.NextLanes.Add(trafficLanesRight[id2]);
                        trafficLanesRight[id2].PrevLanes.Add(trafficLane);
                    }
                }
            }
        }

        private static List<(long, long)> FindLaneletConnections(LaneletMap laneletMap)
        {
            var lanelets = laneletMap.Lanelets.Values;
            return
                (from ll1 in lanelets
                 from ll2 in lanelets
                 where ll2.IsNextTo(ll1)
                 select (ll1.ID, ll2.ID))
                .ToList();
        }

        private void SetRightOfWays()
        {
            // Automatically set right of ways
            var lanes = trafficLanesLeft.Values.ToList();
            foreach (var lane in lanes)
            {
                if (lane.TurnDirection != TrafficLane.TurnDirectionType.STRAIGHT)
                    TrafficLaneEditor.FindAndSetRightOfWays(lane, lanes);
            }

            lanes = trafficLanesRight.Values.ToList();
            foreach (var lane in lanes)
            {
                if (lane.TurnDirection != TrafficLane.TurnDirectionType.STRAIGHT)
                    TrafficLaneEditor.FindAndSetRightOfWays(lane, lanes);
            }
        }
    }
}
