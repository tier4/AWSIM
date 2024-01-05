using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace AWSIM.TrafficSimulation
{
    public enum NPCVehicleSpeedMode
    {
        NORMAL,
        SLOW,
        STOP,
        SUDDEN_STOP,
        ABSOLUTE_STOP
    }

    public enum NPCVehicleYieldPhase
    {
        NONE,
        ENTERING_INTERSECTION,
        AT_INTERSECTION,
        INTERSECTION_BLOCKED,
        LEFT_HAND_RULE_ENTERING_INTERSECTION,
        LEFT_HAND_RULE_AT_INTERSECTION,
        LANES_RULES_ENTERING_INTERSECTION,
        LANES_RULES_AT_INTERSECTION,
        FORCING_PRIORITY
    }

    /// <summary>
    /// Internal state of NPC vehicle updated every frame in <see cref="NPCVehicleSimulator"/>.
    /// </summary>
    public class NPCVehicleInternalState
    {
        // Immutable states
        public NPCVehicle Vehicle { get; private set; }
        public Vector3 FrontCenterLocalPosition { get; private set; }
        public List<TrafficLane> Route { get; set; }
        public Vector3 BackCenterLocalPosition { get; private set; }

        // Output from Cognition (Waypoint Following)
        public IList<TrafficLane> FollowingLanes { get; set; } = new List<TrafficLane>();
        public int WaypointIndex { get; set; }

        // Output from Cognition (Curve)
        public bool IsTurning { get; set; }

        // Output from Cognition (Front Vehicle)
        public float DistanceToFrontVehicle { get; set; }

        // Output from Cognition (Traffic Light)
        public TrafficLane TrafficLightLane { get; set; }
        public TrafficLightPassability TrafficLightPassability { get; set; }

        // Output from Cognition (Right of Way)
        public TrafficLane? YieldLane { get; set; }
        public NPCVehicleYieldPhase YieldPhase { get; set; }
        public Vector3 YieldPoint { get; set; }

        // Output from Decision
        public Vector3 TargetPoint { get; set; }
        public NPCVehicleSpeedMode SpeedMode { get; set; }

        // Output from Control
        public Vector3 Position { get; set; }
        public float Yaw { get; set; }
        public float Speed { get; set; }
        public float YawSpeed { get; set; }
        public float Width { get; set; }

        // Output from any steps
        public bool ShouldDespawn { get; set; }

        // Debugs
        public Transform DominatingVehicle { get; set; }
        public bool IsStoppedByFrontVehicle { get; set; }

        public Vector3 Forward =>
            Quaternion.AngleAxis(Yaw, Vector3.up) * Vector3.forward;

        public Vector3 FrontCenterPosition =>
            Position + Quaternion.AngleAxis(Yaw, Vector3.up) * FrontCenterLocalPosition;

        public Vector3 ExpandedBackCenterPosition(float extensionToRear = 0f)
        {

            var backCenterPositionRaw = BackCenterLocalPosition;
            backCenterPositionRaw.z -= extensionToRear;
            if (Vehicle.TrailerTransform)
            {
                var yaw = Vehicle.TrailerTransform.rotation.eulerAngles.y;
                return Vehicle.TrailerTransform.position + Quaternion.AngleAxis(yaw, Vector3.up) * backCenterPositionRaw;
            }
            else
            {
                return Position + Quaternion.AngleAxis(Yaw, Vector3.up) * backCenterPositionRaw;
            }

        }

        public Vector3 BackCenterPosition => ExpandedBackCenterPosition(0);

        public float DistanceToTargetPoint
            => SignedDistanceToPointOnLane(TargetPoint);

        public float DistanceToCurrentWaypoint
            => SignedDistanceToPointOnLane(CurrentFollowingLane.Waypoints[WaypointIndex]);

        public float DistanceToNextLane
            => SignedDistanceToPointOnLane(CurrentFollowingLane.Waypoints[CurrentFollowingLane.Waypoints.Length - 1]);

        public float DistanceToIntersection
        {
            get
            {
                if (FirstLaneWithIntersection == null) return float.MaxValue;
                return SignedDistanceToPointOnLane(FirstLaneWithIntersection.StopLine != null
                    ? FirstLaneWithIntersection.StopLine.CenterPoint
                    : FirstLaneWithIntersection.Waypoints[0]);
            }
        }

        public bool ObstructedByVehicleBehindIntersection => DistanceToIntersection > DistanceToFrontVehicle;

        private int routeIndex = 0;

        // TODO: Calculate distance along the lane
        public float SignedDistanceToPointOnLane(Vector3 point)
        {
            var position = FrontCenterPosition;
            position.y = 0f;
            point.y = 0f;

            var hasPassedThePoint = Vector3.Dot(Forward, point - position) < 0f;

            var distance = Vector3.Distance(position, point);
            return hasPassedThePoint ? -distance : distance;
        }

        public TrafficLane CurrentFollowingLane
            => FollowingLanes.FirstOrDefault();

        public TrafficLane FirstLaneWithIntersection => FollowingLanes.FirstOrDefault(lane => lane.intersectionLane == true);

        public bool IsNextLaneIntersection()
        {
            if (FollowingLanes.Count > 0)
                return FollowingLanes[1].intersectionLane;
            return false;
        }

        public Vector3? LastIntersectionWaypoint
        {
            get
            {
                if (FirstLaneWithIntersection == null) return null;
                return FirstLaneWithIntersection.Waypoints[FirstLaneWithIntersection.Waypoints.Length - 1];
            }
        }

        public Vector3? FirstIntersectionWaypoint
        {
            get
            {
                if (FirstLaneWithIntersection == null) return null;
                return FirstLaneWithIntersection.Waypoints[0];
            }
        }

        public bool yieldingPriorityAtTrafficLight => (!CurrentFollowingLane.intersectionLane
                    && TrafficLightPassability == TrafficLightPassability.RED);

        public bool isEnteringIntersection => FollowingLanes.Count > 1 && FollowingLanes[1].intersectionLane;

        public bool isOnIntersection => FollowingLanes.Count > 0 && CurrentFollowingLane.intersectionLane;

        public bool isEnteringYieldingLane => FirstLaneWithIntersection?.RightOfWayLanes.Count > 0;

        /// <summary>
        /// Get the next lane of <paramref name="target"/>.<br/>
        /// If <paramref name="target"/> is the last lane, <see cref="FollowingLanes"/> is extended.<br/>
        /// If <paramref name="target"/> does not exist in <see cref="FollowingLanes"/>,<br/>
        /// the vehicle is considered to have passed through <paramref name="target"/> and <see cref="CurrentFollowingLane"/> is returned.
        /// </summary>
        /// <param name="target"></param>
        /// <returns></returns>
        public TrafficLane GetOrExtendNextLane(TrafficLane target)
        {
            var isMatchingLaneFound = false;
            foreach (var lane in FollowingLanes)
            {
                if (isMatchingLaneFound)
                    return lane;
                if (lane == target)
                    isMatchingLaneFound = true;
            }

            if (isMatchingLaneFound)
            {
                return ExtendFollowingLane()
                    ? FollowingLanes.Last()
                    : null;
            }

            return FollowingLanes.First();
        }

        public bool intersectOverall(NPCVehicleInternalState state)
        {
            if (FollowingLanes.Count == 0 && state.FollowingLanes.Count == 0)
                return false;
            if (CurrentFollowingLane.Waypoints.Length == 0 && state.CurrentFollowingLane.Waypoints.Length == 0)
                return false;

            Vector3? refStartNullable = FirstIntersectionWaypoint;
            Vector3? refGoalNullable = LastIntersectionWaypoint;
            Vector3? stateStartNullable = state.FirstIntersectionWaypoint;
            Vector3? stateGoalNullable = state.LastIntersectionWaypoint;
            if (refStartNullable == null || refGoalNullable == null || stateStartNullable == null || stateGoalNullable == null)
                return false;
            if (!CheckIfLinesIntersect(refStartNullable.Value, refGoalNullable.Value, stateStartNullable.Value, stateGoalNullable.Value))
                return false;

            foreach (TrafficLane refLane in FollowingLanes)
            {
                foreach (TrafficLane lane in state.FollowingLanes)
                {
                    var refLaneIndex = (refLane.name == CurrentFollowingLane.name) ? WaypointIndex : 0;
                    var laneIndex = (lane.name == state.CurrentFollowingLane.name) ? state.WaypointIndex : 0;
                    for (int i = refLaneIndex; i < refLane.Waypoints.Length - 1; i++)
                    {
                        for (int j = laneIndex; j < lane.Waypoints.Length - 1; j++)
                        {
                            if (Vector3.Distance(refLane.Waypoints[i + 1], lane.Waypoints[j + 1]) < 1f)
                            {
                                return true;
                            }

                            if (CheckIfLinesIntersect(refLane.Waypoints[i], refLane.Waypoints[i + 1], lane.Waypoints[j], lane.Waypoints[j + 1]))
                            {
                                return true;
                            }
                        }
                    }

                    if (lane.name == state.CurrentFollowingLane.name)
                    {
                        for (int i = refLaneIndex; i < refLane.Waypoints.Length - 1; i++)
                        {
                            if (CheckIfLinesIntersect(refLane.Waypoints[i], refLane.Waypoints[i + 1], state.ExpandedBackCenterPosition(), state.CurrentFollowingLane.Waypoints[state.WaypointIndex]))
                            {
                                return true;
                            }
                        }
                    }

                    if (refLane.name == CurrentFollowingLane.name)
                    {
                        for (int j = laneIndex; j < lane.Waypoints.Length - 1; j++)
                        {
                            if (CheckIfLinesIntersect(ExpandedBackCenterPosition(), CurrentFollowingLane.Waypoints[WaypointIndex], lane.Waypoints[j], lane.Waypoints[j + 1]))
                            {
                                return true;
                            }
                        }
                    }

                    if (lane.intersectionLane)
                        break;
                }
                if (refLane.intersectionLane)
                    break;
            }
            return CheckIfLinesIntersect(ExpandedBackCenterPosition(), CurrentFollowingLane.Waypoints[WaypointIndex], state.ExpandedBackCenterPosition(), state.CurrentFollowingLane.Waypoints[state.WaypointIndex]);


            bool CheckIfLinesIntersect(Vector3 A1, Vector3 B1, Vector3 A2, Vector3 B2, bool verbose = false)
            {
                Vector2 line1point1 = new Vector2(A1.x, A1.z);
                Vector2 line1point2 = new Vector2(B1.x, B1.z);
                Vector2 line2point1 = new Vector2(A2.x, A2.z);
                Vector2 line2point2 = new Vector2(B2.x, B2.z);


                Vector2 a = line1point2 - line1point1;
                Vector2 b = line2point1 - line2point2;
                Vector2 c = line1point1 - line2point1;

                float alphaNumerator = b.y * c.x - b.x * c.y;
                float betaNumerator = a.x * c.y - a.y * c.x;
                float denominator = a.y * b.x - a.x * b.y;

                if (denominator == 0)
                {
                    return false;
                }
                else if (denominator > 0)
                {
                    if (alphaNumerator < 0 || alphaNumerator > denominator || betaNumerator < 0 || betaNumerator > denominator)
                    {
                        return false;
                    }
                }
                else if (alphaNumerator > 0 || alphaNumerator < denominator || betaNumerator > 0 || betaNumerator < denominator)
                {
                    return false;
                }
                return true;
            }
        }
        public bool intersectNow(NPCVehicleInternalState state)
        {
            for (int i = WaypointIndex; i < CurrentFollowingLane.Waypoints.Length - 1; i++)
            {
                for (int j = state.WaypointIndex; j < state.CurrentFollowingLane.Waypoints.Length - 1; j++)
                {
                    if (Vector3.Distance(CurrentFollowingLane.Waypoints[i + 1], state.CurrentFollowingLane.Waypoints[j + 1]) < 1f)
                        return true;

                    if (CheckIfLinesIntersect(CurrentFollowingLane.Waypoints[i], CurrentFollowingLane.Waypoints[i + 1], state.CurrentFollowingLane.Waypoints[j], state.CurrentFollowingLane.Waypoints[j + 1]))
                    {
                        return true;
                    }
                }
            }

            Vector3 PoseA = ExpandedBackCenterPosition(state.Width);
            Vector3 NearestA = CurrentFollowingLane.Waypoints[WaypointIndex]; ;//CurrentFollowingLane.Waypoints[CurrentFollowingLane.Waypoints.Length - 1];
            for (int j = state.WaypointIndex; j < state.CurrentFollowingLane.Waypoints.Length - 1; j++)
            {
                if (Vector3.Distance(NearestA, state.CurrentFollowingLane.Waypoints[j + 1]) < 1f)
                    return true;

                if (CheckIfLinesIntersect(PoseA, NearestA, state.CurrentFollowingLane.Waypoints[j], state.CurrentFollowingLane.Waypoints[j + 1]))
                {
                    return true;
                }
            }

            Vector3 PoseB = state.ExpandedBackCenterPosition(Width);
            Vector3 NearestB = state.CurrentFollowingLane.Waypoints[state.WaypointIndex]; //state.CurrentFollowingLane.Waypoints[state.CurrentFollowingLane.Waypoints.Length - 1];
            for (int i = WaypointIndex; i < CurrentFollowingLane.Waypoints.Length - 1; i++)
            {
                if (Vector3.Distance(CurrentFollowingLane.Waypoints[i + 1], NearestB) < 1f)
                    return true;

                if (CheckIfLinesIntersect(CurrentFollowingLane.Waypoints[i], CurrentFollowingLane.Waypoints[i + 1], PoseB, NearestB))
                {
                    return true;
                }
            }

            return CheckIfLinesIntersect(PoseA, NearestA, PoseB, NearestB);

            bool CheckIfLinesIntersect(Vector3 A1, Vector3 B1, Vector3 A2, Vector3 B2, bool verbose = false)
            {
                Vector2 line1point1 = new Vector2(A1.x, A1.z);
                Vector2 line1point2 = new Vector2(B1.x, B1.z);
                Vector2 line2point1 = new Vector2(A2.x, A2.z);
                Vector2 line2point2 = new Vector2(B2.x, B2.z);

                Vector2 a = line1point2 - line1point1;
                Vector2 b = line2point1 - line2point2;
                Vector2 c = line1point1 - line2point1;

                float alphaNumerator = b.y * c.x - b.x * c.y;
                float betaNumerator = a.x * c.y - a.y * c.x;
                float denominator = a.y * b.x - a.x * b.y;

                if (denominator == 0)
                {
                    return false;
                }
                else if (denominator > 0)
                {
                    if (alphaNumerator < 0 || alphaNumerator > denominator || betaNumerator < 0 || betaNumerator > denominator)
                    {
                        return false;
                    }
                }
                else if (alphaNumerator > 0 || alphaNumerator < denominator || betaNumerator > 0 || betaNumerator < denominator)
                {
                    return false;
                }
                return true;
            }
        }
        public bool intersectNowFront(NPCVehicleInternalState state)
        {
            for (int i = WaypointIndex; i < CurrentFollowingLane.Waypoints.Length - 1; i++)
            {
                for (int j = state.WaypointIndex; j < state.CurrentFollowingLane.Waypoints.Length - 1; j++)
                {
                    if (Vector3.Distance(CurrentFollowingLane.Waypoints[i + 1], state.CurrentFollowingLane.Waypoints[j + 1]) < 1f)
                    {
                        return true;
                    }

                    if (CheckIfLinesIntersect(CurrentFollowingLane.Waypoints[i], CurrentFollowingLane.Waypoints[i + 1], state.CurrentFollowingLane.Waypoints[j], state.CurrentFollowingLane.Waypoints[j + 1]))
                    {
                        return true;
                    }
                }
            }

            Vector3 PoseA = FrontCenterPosition;
            Vector3 NearestA = CurrentFollowingLane.Waypoints[WaypointIndex];
            for (int j = state.WaypointIndex; j < state.CurrentFollowingLane.Waypoints.Length - 1; j++)
            {
                if (Vector3.Distance(NearestA, state.CurrentFollowingLane.Waypoints[j + 1]) < 1f)
                {
                    return true;
                }

                if (CheckIfLinesIntersect(PoseA, NearestA, state.CurrentFollowingLane.Waypoints[j], state.CurrentFollowingLane.Waypoints[j + 1]))
                {
                    return true;
                }
            }

            Vector3 PoseB = state.ExpandedBackCenterPosition(Width);
            Vector3 NearestB = state.CurrentFollowingLane.Waypoints[state.WaypointIndex];
            for (int i = WaypointIndex; i < CurrentFollowingLane.Waypoints.Length - 1; i++)
            {
                if (Vector3.Distance(CurrentFollowingLane.Waypoints[i + 1], NearestB) < 1f)
                {
                    return true;
                }

                if (CheckIfLinesIntersect(CurrentFollowingLane.Waypoints[i], CurrentFollowingLane.Waypoints[i + 1], PoseB, NearestB))
                {
                    return true;
                }
            }

            return CheckIfLinesIntersect(PoseA, NearestA, PoseB, NearestB);

            bool CheckIfLinesIntersect(Vector3 A1, Vector3 B1, Vector3 A2, Vector3 B2, bool verbose = false)
            {
                Vector2 line1point1 = new Vector2(A1.x, A1.z);
                Vector2 line1point2 = new Vector2(B1.x, B1.z);
                Vector2 line2point1 = new Vector2(A2.x, A2.z);
                Vector2 line2point2 = new Vector2(B2.x, B2.z);

                Vector2 a = line1point2 - line1point1;
                Vector2 b = line2point1 - line2point2;
                Vector2 c = line1point1 - line2point1;

                float alphaNumerator = b.y * c.x - b.x * c.y;
                float betaNumerator = a.x * c.y - a.y * c.x;
                float denominator = a.y * b.x - a.x * b.y;

                if (denominator == 0)
                {
                    return false;
                }
                else if (denominator > 0)
                {
                    if (alphaNumerator < 0 || alphaNumerator > denominator || betaNumerator < 0 || betaNumerator > denominator)
                    {
                        return false;
                    }
                }
                else if (alphaNumerator > 0 || alphaNumerator < denominator || betaNumerator > 0 || betaNumerator < denominator)
                {
                    return false;
                }
                return true;
            }
        }

        public bool ExtendFollowingLane()
        {
            // If the internal state has route - use it. Otherwise, choose next lane randomly.
            var lastLane = FollowingLanes.Last();
            TrafficLane nextLane;
            if (Route == null || Route.Count == 0 || routeIndex + 1 == Route.Count)
            {
                nextLane = RandomTrafficUtils.GetRandomElement(lastLane.NextLanes);
            }
            else
            {
                // Todo: check if route[0] equals to the spawnlane
                // Todo: check if next lane in route is valid (is one of lastLane.NextLanes)
                routeIndex += 1;
                nextLane = Route[routeIndex];
            }
            if (nextLane == null)
                return false;
            FollowingLanes.Add(nextLane);
            return true;
        }

        public void RemoveCurrentFollowingLane()
        {
            FollowingLanes.RemoveAt(0);
        }

        public static NPCVehicleInternalState Create(NPCVehicle vehicle, TrafficLane lane, int waypointIndex = 0)
        {
            var state = new NPCVehicleInternalState
            {
                Vehicle = vehicle,
                Position = vehicle.transform.position,
                Yaw = vehicle.transform.rotation.eulerAngles.y,
                WaypointIndex = waypointIndex,
                TargetPoint = lane.Waypoints[waypointIndex],
                FrontCenterLocalPosition = new Vector3
                {
                    x = 0f,
                    y = 0f,
                    z = vehicle.Bounds.max.z
                },
                BackCenterLocalPosition = new Vector3
                {
                    x = 0f,
                    y = 0f,
                    z = vehicle.Bounds.min.z
                },
                Width = vehicle.Bounds.size.x
            };
            state.FollowingLanes.Add(lane);
            return state;
        }

        public static NPCVehicleInternalState Create(NPCVehicle vehicle, List<TrafficLane> route, int waypointIndex = 0)
        {
            var state = NPCVehicleInternalState.Create(vehicle, route.First(), waypointIndex);
            state.Route = route;
            return state;
        }
    }
}
