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

        public Vector3 BackCenterPosition => ExpandedBackCenterPosition(0f);

        public float DistanceToTargetPoint
            => SignedDistanceToPointOnLane(TargetPoint);

        public Vector3 CurrentWaypoint => CurrentFollowingLane.Waypoints[WaypointIndex];

        public float DistanceToCurrentWaypoint
            => SignedDistanceToPointOnLane(CurrentWaypoint);

        public float DistanceToNextLane
            => CalculateDistanceToNextLane();
        public float DistanceToIntersection
            => FirstLaneWithIntersection == null ? float.MaxValue
            : DistanceToClosestTrafficLane();

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

        private float CalculateDistanceToNextLane()
        {
            var nextLane = CurrentFollowingLane;
            if (nextLane == null || nextLane.Waypoints == null || nextLane.Waypoints.Length == 0)
            {
                return float.MaxValue;
            }
            var vehiclePosition = FrontCenterPosition;
            RandomTrafficUtils.GetLaneFollowingProgressAndLaneLength(
                vehiclePosition,
                nextLane,
                out var laneFollowingProgress,
                out var laneLenght);
            return (1f - laneFollowingProgress) * laneLenght;
        }
        
        public float DistanceToClosestTrafficLane()
        {
            if (TrafficLightLane is null && !FollowingLanes.Contains(TrafficLightLane))
            {
                return float.MaxValue;
            }

            var distance = 0f;
            var vehiclePosition = FrontCenterPosition;
            bool startAddingWholeLanesDistance = false;

            for (var i = 0; i < FollowingLanes.Count; i++)
            {
                RandomTrafficUtils.GetLaneFollowingProgressAndLaneLength(
                    vehiclePosition,
                    FollowingLanes[i],
                    out var laneFollowingProgress,
                    out var laneLenght);
                if (!startAddingWholeLanesDistance)
                {
                    //vehicle is before first not-skipped lane
                    if (laneFollowingProgress <= 0f)
                    {
                        distance += laneLenght;
                        startAddingWholeLanesDistance = true;
                    }
                    //vehicle is on lane
                    else if (laneFollowingProgress < 1f)
                    {
                        var progressToLaneEnd = (1 - laneFollowingProgress);
                        distance += laneLenght * progressToLaneEnd;
                        startAddingWholeLanesDistance = true;
                    }
                }
                else
                {
                    //when distance was calculated once partially/for whole lane it keeps calculating whole lanes
                    //until meeting traffic light lane
                    distance += laneLenght;
                }

                //if traffic lane, stop computing
                if (FollowingLanes[i] == TrafficLightLane)
                {
                    break;
                }
            }
            return distance;
        }

        public TrafficLane CurrentFollowingLane => FollowingLanes.FirstOrDefault();

        public TrafficLane FirstLaneWithIntersection => FollowingLanes.FirstOrDefault(lane => lane.intersectionLane == true);

        public bool IsNextLaneIntersection => FollowingLanes.Count > 0 && FollowingLanes[1].intersectionLane;

        public Vector3? LastIntersectionWaypoint
            => FirstLaneWithIntersection?.Waypoints?.Any() != true ? (Vector3?)null
            : FirstLaneWithIntersection.Waypoints.Last();

        public Vector3? FirstIntersectionWaypoint
            => FirstLaneWithIntersection?.Waypoints?.Any() != true ? (Vector3?)null
            : FirstLaneWithIntersection.Waypoints.First();

        public bool yieldingPriorityAtTrafficLight => (!CurrentFollowingLane.intersectionLane
                    && TrafficLightPassability == TrafficLightPassability.RED);

        public bool isOnIntersection => FollowingLanes.Count > 0 && CurrentFollowingLane.intersectionLane;

        public bool isIntersectionWithYieldingLane => FirstLaneWithIntersection?.RightOfWayLanes.Count > 0;


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
