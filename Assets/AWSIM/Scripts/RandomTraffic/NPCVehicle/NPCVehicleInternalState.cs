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
        ENTERING_YIELDING_LANE,
        YIELDING,
        ON_YIELDING_LANE
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
        public TrafficLane YieldLane { get; set; }
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

        // Output from any steps
        public bool ShouldDespawn { get; set; }

        // Debugs
        public Transform DominatingVehicle { get; set; }
        public bool IsStoppedByFrontVehicle { get; set; }

        public Vector3 Forward =>
            Quaternion.AngleAxis(Yaw, Vector3.up) * Vector3.forward;

        public Vector3 FrontCenterPosition =>
            Position + Quaternion.AngleAxis(Yaw, Vector3.up) * FrontCenterLocalPosition;

        public Vector3 BackCenterPosition =>
            Position + Quaternion.AngleAxis(Yaw, Vector3.up) * BackCenterLocalPosition;

        public float DistanceToTargetPoint
            => SignedDistanceToPointOnLane(TargetPoint);

        public float DistanceToCurrentWaypoint
            => SignedDistanceToPointOnLane(CurrentFollowingLane.Waypoints[WaypointIndex]);

        public float DistanceToNextLane
            => SignedDistanceToPointOnLane(CurrentFollowingLane.Waypoints[CurrentFollowingLane.Waypoints.Length - 1]);

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
            if(Route == null || Route.Count == 0 || routeIndex + 1 == Route.Count)
            {
                nextLane = RandomTrafficUtils.GetRandomElement(lastLane.NextLanes);
            } else {
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
                }
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
