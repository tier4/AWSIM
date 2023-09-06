using AWSIM.Lanelet;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;
using GeometryUtility = AWSIM.Lanelet.GeometryUtility;

namespace AWSIM.TrafficSimulation
{
    /// <summary>
    /// A class that provides capability to convert Lanelet2 data into environment components.
    /// </summary>
    public class LaneletLoader
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
        private readonly Dictionary<long, TrafficLane> trafficLanes = new Dictionary<long, TrafficLane>();
        private readonly Dictionary<long, StopLine> stopLines = new Dictionary<long, StopLine>();

        private LaneletMap laneletMap;
        private GameObject trafficLaneHolder;
        private GameObject stopLineHolder;
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

            CreateStopLines();
            SetStopSigns();
            SetTrafficLightsOfStopLines();

            SetStopLineOfLanes();

            AssignLaneletElementIdToTrafficSignalGameObjects();
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
                var waypoints = lanelet.CalculateCenterline(settings.Resolution, settings.MinDeltaLength, settings.MinDeltaAngle);
                lanelet.Attributes.TryGetValue(AttributeKeys.TurnDirection, out string turnDirectionString);
                var turnDirection =
                    turnDirectionString == AttributeValues.Straight ? TrafficLane.TurnDirectionType.STRAIGHT :
                    turnDirectionString == AttributeValues.Left ? TrafficLane.TurnDirectionType.LEFT :
                    turnDirectionString == AttributeValues.Right ? TrafficLane.TurnDirectionType.RIGHT
                    : TrafficLane.TurnDirectionType.STRAIGHT;
                var trafficLane = TrafficLane.Create(waypoints, turnDirection, speedLimitMps);
                trafficLane.transform.parent = trafficLaneHolder.transform;
                trafficLanes.Add(lanelet.ID, trafficLane);
            }
        }

        private void SetTrafficLaneConnections()
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
            var lanes = trafficLanes.Values.ToList();
            foreach (var lane in lanes)
            {
                if (lane.TurnDirection != TrafficLane.TurnDirectionType.STRAIGHT)
                    TrafficLaneEditor.FindAndSetRightOfWays(lane, lanes);
            }
        }

        private void SetStopLineOfLanes()
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

                    var stopLineID = regElem.RefLines[0].ID;
                    if (stopLines.ContainsKey(stopLineID))
                    {
                        var stopLine = stopLines[stopLineID];
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

        private void CreateStopLines()
        {
            foreach (var entry in laneletMap.Lines)
            {
                var line = entry.Value;
                if (line.Attributes.TryGetValue(AttributeKeys.Type, out string laneType))
                {
                    if (laneType == AttributeValues.StopLine)
                    {
                        var stopLine = StopLine.Create(line[0], line[1]);
                        stopLine.transform.parent = stopLineHolder.transform;
                        stopLines.Add(entry.Key, stopLine);
                    }
                } else {
                    continue;
                }
            }
        }

        private void SetStopSigns()
        {
            var lines =
                laneletMap.RegulatoryElements.Values
                .Where(regElem => regElem.Type == RegulatoryElementType.TRAFFIC_SIGN)
                .SelectMany(regElem => regElem.RefLines);
            foreach (var line in lines)
            {
                stopLines[line.ID].HasStopSign = true;
            }
        }

        private void SetTrafficLightsOfStopLines()
        {
            var trafficLights = GameObject.FindObjectsOfType<TrafficLight>();

            var regElems =
                laneletMap.RegulatoryElements.Values
                    .Where(regElem => regElem.Type == RegulatoryElementType.TRAFFIC_LIGHT);
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
                        stopLines[line.ID].TrafficLight = trafficLight;
                        break;
                    }

                    if (stopLines[line.ID].TrafficLight == null)
                    {
                        Debug.Log($"No TrafficLight components exist in the scene for the stop line(ID = {line.ID}).");
                    }
                }
            }
        }

        private static TrafficLight FindClosestTrafficLight(TrafficLight[] trafficLights, Vector3 position, float radius = 3f)
        {
            var candidates = trafficLights
                .Where(trafficLight =>
                {
                    var distance2D = GeometryUtility.Distance2D(trafficLight.transform.position, position);
                    return distance2D <= radius;
                }).ToList();
            if (!candidates.Any())
                return null;
            return candidates.Aggregate((closest, next) =>
            {
                if (closest == null)
                    return next;
                var closestDistance = GeometryUtility.Distance2D(closest.transform.position, position);
                var nextDistance = GeometryUtility.Distance2D(next.transform.position, position);
                return closestDistance < nextDistance
                    ? closest
                    : next;
            });
        }

        private void AssignLaneletElementIdToTrafficSignalGameObjects()
        {
            TrafficLight[] trafficLights = GameObject.FindObjectsOfType<TrafficLight>();
            var regElems = laneletMap.RegulatoryElements.Values
                    .Where(regElem => regElem.Type == RegulatoryElementType.TRAFFIC_LIGHT);

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
                    FillTrafficLightRelationIDWayID(closestTrafficLight, regElem.ID, line.ID);
                }
            }
        }

        private void FillTrafficLightRelationIDWayID(TrafficLight trafficLight, long relationId, long wayID)
        {
            EditorUtility.SetDirty(trafficLight);
            Undo.RecordObject(trafficLight, "Assigning lanelet id");
            var trafficLightLaneletID = trafficLight.GetComponentInParent<TrafficLightLaneletID>();
            if (trafficLightLaneletID == null)
            {
                trafficLight.gameObject.AddComponent<TrafficLightLaneletID>();
                trafficLightLaneletID = trafficLight.GetComponentInParent<TrafficLightLaneletID>();
            }

            if (!trafficLightLaneletID.relationID.Contains(relationId))
            {
                trafficLightLaneletID.relationID.Add(relationId);
            }
            trafficLightLaneletID.wayID = wayID;
            PrefabUtility.RecordPrefabInstancePropertyModifications(trafficLight);
        }
    }
}
