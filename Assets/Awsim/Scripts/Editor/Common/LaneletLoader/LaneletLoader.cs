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
using UnityEditor;
using UnityEngine;
using Awsim.Entity;
using Awsim.Usecase.TrafficSimulation;

namespace Awsim.Common
{
    /// <summary>
    /// A class that provides capability to convert Lanelet2 data into environment components.
    /// </summary>
    public class LaneletLoader
    {
        [System.Serializable]
        public struct WaypointSettings
        {
            /// <summary>
            /// Resolution of resampling. Lower values provide better accuracy at the cost of processing time.
            /// </summary>
            public float Resolution => _resolution;

            /// <summary>
            /// Minimum length(m) between adjacent points.
            /// </summary>
            public float MinDeltaLength => _minDeltaLength;

            /// <summary>
            /// Minimum angle(deg) between adjacent edges.
            /// </summary>
            public float MinDeltaAngle => _minDeltaAngle;

            [SerializeField, Range(0.1f, 2f)]
            [Tooltip("Resolution of resampling. Lower values provide better accuracy at the cost of processing time.")]
            float _resolution;
            [SerializeField, Range(5f, 100f)]
            [Tooltip("Minimum length(m) between adjacent points.")]
            float _minDeltaLength;
            [SerializeField, Range(5f, 30f)]
            [Tooltip("Minimum angle(deg) between adjacent edges.")]
            float _minDeltaAngle;

            public static WaypointSettings Default()
            {
                return new WaypointSettings(1f, 30f, 15f);
            }

            public WaypointSettings(float resolution, float minDeltaLength, float minDeltaAngle)
            {
                this._resolution = resolution;
                this._minDeltaLength = minDeltaLength;
                this._minDeltaAngle = minDeltaAngle;
            }
        }

        // Internal variables for mapping lanelet id to components
        readonly Dictionary<long, TrafficLane> trafficLanes = new Dictionary<long, TrafficLane>();
        readonly Dictionary<long, StopLine> stopLines = new Dictionary<long, StopLine>();

        LaneletMap laneletMap;
        GameObject trafficLaneHolder;
        GameObject stopLineHolder;
        WaypointSettings settings = WaypointSettings.Default();

        static List<(long, long)> FindLaneletConnections(LaneletMap laneletMap)
        {
            var lanelets = laneletMap.Lanelets.Values;
            return
                (from ll1 in lanelets
                 from ll2 in lanelets
                 where ll2.IsNextTo(ll1)
                 select (ll1.Id, ll2.Id))
                .ToList();
        }

        static LaneletTrafficLight FindClosestTrafficLight(LaneletTrafficLight[] trafficLights, Vector3 position, float radius = 3f)
        {
            var candidates = trafficLights
                .Where(trafficLight =>
                {
                    var distance2D = LaneletGeometryUtility.Distance2D(trafficLight.transform.position, position);
                    return distance2D <= radius;
                }).ToList();
            if (!candidates.Any())
                return null;
            return candidates.Aggregate((closest, next) =>
            {
                if (closest == null)
                    return next;
                var closestDistance = LaneletGeometryUtility.Distance2D(closest.transform.position, position);
                var nextDistance = LaneletGeometryUtility.Distance2D(next.transform.position, position);
                return closestDistance < nextDistance
                    ? closest
                    : next;
            });
        }

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
            trafficLaneHolder = new GameObject("TrafficLanes");
            stopLineHolder = new GameObject("StopLines");
            if (holder != null)
            {
                trafficLaneHolder.transform.parent = holder.transform;
                stopLineHolder.transform.parent = holder.transform;
            }

            CreateTrafficLanes();
            SetTrafficLaneConnections();
            SetRightOfWays();
            SetIntersectionLane();

            CreateStopLines();
            SetStopSigns();
            SetTrafficLightsOfStopLines();

            SetStopLineOfLanes();

            AssignLaneletElementIdToTrafficSignalGameObjects();
        }

        void CreateTrafficLanes()
        {
            foreach (var lanelet in laneletMap.Lanelets.Values)
            {
                if (lanelet.Attributes[LaneletAttributeKey.Subtype] != LaneletAttributeValue.Road)
                {
                    continue;
                }

                lanelet.Attributes.TryGetValue(LaneletAttributeKey.SpeedLimit, out float speedLimitKph);
                // Convert km/h -> m/s
                var speedLimitMps = speedLimitKph / 3600f * 1000f;
                var waypoints = lanelet.CalculateCenterline(settings.Resolution, settings.MinDeltaLength, settings.MinDeltaAngle);
                lanelet.Attributes.TryGetValue(LaneletAttributeKey.TurnDirection, out string turnDirectionString);
                var turnDirection =
                    turnDirectionString == LaneletAttributeValue.Straight ? TrafficLane.TurnDirectionType.Straight :
                    turnDirectionString == LaneletAttributeValue.Left ? TrafficLane.TurnDirectionType.Left :
                    turnDirectionString == LaneletAttributeValue.Right ? TrafficLane.TurnDirectionType.Right
                    : TrafficLane.TurnDirectionType.Straight;
                var trafficLane = TrafficLane.Create(waypoints, turnDirection, speedLimitMps);
                trafficLane.transform.parent = trafficLaneHolder.transform;
                trafficLane.name = "TrafficLane." + (int)lanelet.Id;
                trafficLanes.Add(lanelet.Id, trafficLane);
            }
        }

        void SetTrafficLaneConnections()
        {
            var connections = FindLaneletConnections(laneletMap);
            foreach (var entry in trafficLanes)
            {
                var id = entry.Key;
                var trafficLane = entry.Value;
                foreach (var (id1, id2) in connections)
                {
                    if (id == id1 && trafficLanes.ContainsKey(id2))
                    {
                        trafficLane.NextLanes.Add(trafficLanes[id2]);
                        trafficLanes[id2].PrevLanes.Add(trafficLane);
                    }
                }
            }
        }


        void SetRightOfWays()
        {
            // Automatically set right of ways
            var lanes = trafficLanes.Values.ToList();
            foreach (var lane in lanes)
            {
                if (lane.TurnDirection != TrafficLane.TurnDirectionType.Straight)
                    TrafficLaneEditor.FindAndSetRightOfWays(lane, lanes);
            }
        }

        void SetIntersectionLane()
        {
            foreach (TrafficLane lane in trafficLanes.Values)
            {
                if (lane == null)
                {
                    Debug.LogWarning($"Found GameObject ${lane.name} without TrafficLane script.");
                    continue;
                }
                // If refTrafficLane is already an intersectionLane, skip this TrafficLane
                else if (lane._intersectionLane == true)
                {
                    continue;
                }
                // If TrafficLane has RightOfWayLanes, set it as an intersectionLane and skip this TrafficLane
                else if (lane.RightOfWayLanes != null && lane.RightOfWayLanes.Count > 0)
                {
                    lane._intersectionLane = true;
                    continue;
                }
                // Else search for otherTrafficLane that has refTrafficLane in RightOfWayLanes, if found, assign intersectionLane as true and break
                else
                {
                    foreach (TrafficLane otherLane in trafficLanes.Values)
                    {
                        if (otherLane == null)
                        {
                            Debug.LogWarning($"Found GameObject ${otherLane.name} without TrafficLane script.");
                            continue;
                        }

                        foreach (TrafficLane otherRightOfLane in otherLane.RightOfWayLanes)
                        {
                            if (otherRightOfLane != null && otherRightOfLane.name == lane.name)
                            {
                                lane._intersectionLane = true;
                                otherLane._intersectionLane = true;
                                break;
                            }
                        }
                        // If refTrafficLane is already an intersectionLane -> interrupt further search
                        if (lane._intersectionLane == true)
                            break;
                    }
                }
            }
        }

        void SetStopLineOfLanes()
        {
            foreach (var entry in laneletMap.Lanelets)
            {
                var lanelet = entry.Value;
                var id = entry.Key;

                // Assuming that one lanelet has at most one ref_line(e.g. stop line).
                foreach (var regElem in lanelet.RegulatoryElements)
                {
                    if (regElem.RefLines == null || regElem.RefLines.Length == 0)
                        continue;

                    var stopLineId = regElem.RefLines[0].Id;
                    if (stopLines.ContainsKey(stopLineId))
                    {
                        var stopLine = stopLines[stopLineId];
                        // If the stop line is at the first waypoint, it is associated with the previous lane.
                        var distanceToStopLine =
                            HandleUtility.DistancePointLine(
                                trafficLanes[id].Waypoints[0],
                                stopLine.Points[0],
                                stopLine.Points[1]);
                        if (distanceToStopLine < 1f)
                        {
                            foreach (var prevLane in trafficLanes[id].PrevLanes)
                            {
                                prevLane.StopLine = stopLine;
                            }

                            continue;
                        }
                        trafficLanes[id].StopLine = stopLine;
                    }

                    break;
                }
            }
        }

        void CreateStopLines()
        {
            foreach (var entry in laneletMap.Lines)
            {
                var line = entry.Value;
                if (line.Attributes.TryGetValue(LaneletAttributeKey.Type, out string laneType))
                {
                    if (laneType == LaneletAttributeValue.StopLine)
                    {
                        var stopLine = StopLine.Create(line[0], line[1]);
                        stopLine.transform.parent = stopLineHolder.transform;
                        stopLines.Add(entry.Key, stopLine);
                    }
                }
                else
                {
                    continue;
                }
            }
        }

        void SetStopSigns()
        {
            var lines =
                laneletMap.RegulatoryElements.Values
                .Where(regElem => regElem.Type == LaneletRegulatoryElementType.TrafficSign)
                .SelectMany(regElem => regElem.RefLines);
            foreach (var line in lines)
            {
                stopLines[line.Id].HasStopSign = true;
            }
        }

        void SetTrafficLightsOfStopLines()
        {
            LaneletTrafficLight[] trafficLights = GameObject.FindObjectsByType<LaneletTrafficLight>(FindObjectsSortMode.InstanceID);

            var regElems =
                laneletMap.RegulatoryElements.Values
                    .Where(regElem => regElem.Type == LaneletRegulatoryElementType.TrafficLight);
            foreach (var regElem in regElems)
            {
                foreach (var line in regElem.RefLines)
                {
                    foreach (var lineOfBulbs in regElem.LightBulbs)
                    {
                        // Select a center light bulb position of the traffic light.
                        var trafficLightPosition = lineOfBulbs.Points[1];
                        // Find TrafficLight component whose position is the same as lanelet data.
                        var trafficLight = FindClosestTrafficLight(trafficLights, trafficLightPosition);
                        if (trafficLight == null)
                            continue;
                        stopLines[line.Id].TrafficLight = trafficLight;
                        break;
                    }

                    if (stopLines[line.Id].TrafficLight == null)
                    {
                        Debug.Log($"No TrafficLight components exist in the scene for the stop line(Id = {line.Id}).");
                    }
                }
            }
        }


        void AssignLaneletElementIdToTrafficSignalGameObjects()
        {
            LaneletTrafficLight[] trafficLights = GameObject.FindObjectsByType<LaneletTrafficLight>(FindObjectsSortMode.InstanceID);
            Dictionary<string, List<long>> verifiedTrafficLights = new Dictionary<string, List<long>>();
            var regElems = laneletMap.RegulatoryElements.Values
                    .Where(regElem => regElem.Type == LaneletRegulatoryElementType.TrafficLight);

            foreach (var regElem in regElems)
            {
                foreach (var line in regElem.Refers)
                {
                    var trafficLightPosition = line.Points[1];
                    var closestTrafficLight = FindClosestTrafficLight(trafficLights, trafficLightPosition);
                    if (closestTrafficLight == null)
                    {
                        continue;
                    }
                    FillTrafficLightRelationIdWayId(closestTrafficLight, regElem.Id, line.Id);
                    if (verifiedTrafficLights.ContainsKey(closestTrafficLight.name))
                    {
                        if (!verifiedTrafficLights[closestTrafficLight.name].Contains(line.Id))
                        {
                            verifiedTrafficLights[closestTrafficLight.name].Add(line.Id);
                        }
                    }
                    else
                    {
                        verifiedTrafficLights.Add(closestTrafficLight.name, new List<long> { line.Id });
                    }

                }
            }

            foreach (var entry in verifiedTrafficLights)
            {
                if (entry.Value.Count >= 2)
                {
                    string wayIds = "";
                    foreach (var wayId in entry.Value)
                    {
                        wayIds += $"{wayId}, ";
                    }
                    Debug.LogWarning($"Verify '{entry.Key}' manually because may include wrong WayId and RelationId. Possible Way Ids [{wayIds}]");
                }
            }
        }

        void FillTrafficLightRelationIdWayId(LaneletTrafficLight trafficLight, long relationId, long wayId)
        {
            EditorUtility.SetDirty(trafficLight);
            Undo.RecordObject(trafficLight, "Assigning lanelet id");
            var trafficLightLaneletId = trafficLight.LaneletId;
            if (trafficLightLaneletId.wayId != LaneletTrafficLight.TrafficLightLaneletId.InitWayId && trafficLightLaneletId.wayId != wayId)
            {
                trafficLightLaneletId.relationId.Clear();
            }
            if (!trafficLightLaneletId.relationId.Contains(relationId))
            {
                trafficLightLaneletId.relationId.Add(relationId);
            }
            trafficLightLaneletId.wayId = wayId;
            PrefabUtility.RecordPrefabInstancePropertyModifications(trafficLight);
        }
    }
}
