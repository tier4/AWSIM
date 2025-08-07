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
using UnityEngine;

namespace Awsim.Usecase.TrafficSimulation
{
    public enum NpcVehicleSpeedMode
    {
        Normal,
        Slow,
        Stop,
        SuddenStop,
        AbsoluteStop
    }

    public enum NpcVehicleYieldPhase
    {
        None,
        EnteringIntersection,
        AtIntersection,
        IntersectionBlocked,
        LeftHandRuleEnteringIntersection,
        LeftHandRuleAtIntersection,
        LanesRulesEnteringIntersection,
        LanesRulesAtIntersection,
        ForcingPriority
    }

    /// <summary>
    /// Internal state of NPC vehicle updated every frame in <see cref="NpcVehicleSimulator"/>.
    /// </summary>
    public class NpcVehicleInternalState
    {
        // Immutable states
        public TrafficSimNpcVehicle Vehicle { get; private set; }
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
        public NpcVehicleYieldPhase YieldPhase { get; set; }
        public Vector3 YieldPoint { get; set; }

        // Output from Decision
        public Vector3 TargetPoint { get; set; }
        public NpcVehicleSpeedMode SpeedMode { get; set; }

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
            // if (Vehicle.TrailerTransform)
            // {
            //     var yaw = Vehicle.TrailerTransform.rotation.eulerAngles.y;
            //     return Vehicle.TrailerTransform.position + Quaternion.AngleAxis(yaw, Vector3.up) * backCenterPositionRaw;
            // }
            // else
            // {
            return Position + Quaternion.AngleAxis(Yaw, Vector3.up) * backCenterPositionRaw;
            // }
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
            : DistanceToClosestTrafficLightLane();

        public bool ObstructedByVehicleBehindIntersection => DistanceToIntersection > DistanceToFrontVehicle;

        public TrafficLane CurrentFollowingLane => FollowingLanes.FirstOrDefault();

        public TrafficLane FirstLaneWithIntersection => FollowingLanes.FirstOrDefault(lane => lane._intersectionLane == true);

        public bool IsNextLaneIntersection => FollowingLanes.Count > 0 && FollowingLanes[1]._intersectionLane;

        public Vector3? LastIntersectionWaypoint
            => FirstLaneWithIntersection?.Waypoints?.Any() != true ? (Vector3?)null
            : FirstLaneWithIntersection.Waypoints.Last();

        public Vector3? FirstIntersectionWaypoint
            => FirstLaneWithIntersection?.Waypoints?.Any() != true ? (Vector3?)null
            : FirstLaneWithIntersection.Waypoints.First();

        public bool yieldingPriorityAtTrafficLight => (!CurrentFollowingLane._intersectionLane
                    && TrafficLightPassability == TrafficLightPassability.Red);

        public bool isOnIntersection => FollowingLanes.Count > 0 && CurrentFollowingLane._intersectionLane;

        public bool isIntersectionWithYieldingLane => FirstLaneWithIntersection?.RightOfWayLanes.Count > 0;

        int routeIndex = 0;

        public static NpcVehicleInternalState Create(TrafficSimNpcVehicle vehicle, TrafficLane lane, int waypointIndex = 0)
        {
            var state = new NpcVehicleInternalState
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
                    z = vehicle.NpcVehicle.Bounds.max.z
                },
                BackCenterLocalPosition = new Vector3
                {
                    x = 0f,
                    y = 0f,
                    z = vehicle.NpcVehicle.Bounds.min.z
                },
                Width = vehicle.NpcVehicle.Bounds.size.x
            };
            state.FollowingLanes.Add(lane);
            return state;
        }

        public static NpcVehicleInternalState Create(TrafficSimNpcVehicle vehicle, List<TrafficLane> route, int waypointIndex = 0)
        {
            var state = NpcVehicleInternalState.Create(vehicle, route.First(), waypointIndex);
            state.Route = route;
            return state;
        }

        public float SignedDistanceToPointOnLane(Vector3 point)
        {
            var position = FrontCenterPosition;
            position.y = 0f;
            point.y = 0f;

            var hasPassedThePoint = Vector3.Dot(Forward, point - position) < 0f;

            var distance = Vector3.Distance(position, point);
            return hasPassedThePoint ? -distance : distance;
        }


        public float DistanceToClosestTrafficLightLane()
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
                    distance += laneLenght;
                }

                if (FollowingLanes[i] == TrafficLightLane)
                {
                    break;
                }
            }
            return distance;
        }

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


        float CalculateDistanceToNextLane()
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
    }
}
