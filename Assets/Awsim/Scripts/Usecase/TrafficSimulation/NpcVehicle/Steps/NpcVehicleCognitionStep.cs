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

using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using UnityEngine.Profiling;
using Awsim.Common;

namespace Awsim.Usecase.TrafficSimulation
{
    /// <summary>
    /// Cognition step implementation for a NPC vehicle simulation.
    /// This step checks the followings:<br/>
    /// - Ground existence<br/>
    /// - Next lane and waypoint to follow<br/>
    /// - Traffic light states<br/>
    /// - Traffic conditions(EGOVehicle and NPCVehicle) on right of ways<br/>
    /// - Sharp curves to be passed slowly<br/>
    /// - Front obstacles to be kept distance
    /// </summary>
    public class NpcVehicleCognitionStep : IDisposable
    {
        /// <summary>
        /// NativeContainer compatible data of NPC vehicles for JobSystem input.
        /// </summary>
        struct NativeState
        {
            public Vector3 FrontCenterPosition;
            public float Yaw;
            public int WaypointCount;

            public static NativeState Create(NpcVehicleInternalState state, int waypointCount)
            {
                return new NativeState
                {
                    FrontCenterPosition = state.FrontCenterPosition,
                    Yaw = state.Yaw,
                    WaypointCount = waypointCount
                };
            }
        }

        /// <summary>
        /// Check next lane and waypoint to follow.
        /// </summary>
        struct NextWaypointCheckJob
        {
            // In/Out
            public IReadOnlyList<NpcVehicleInternalState> States;

            public void Execute()
            {
                foreach (var state in States)
                {
                    var isCloseToTarget = state.DistanceToCurrentWaypoint <= 1f;

                    if (!isCloseToTarget)
                        continue;

                    if (state.WaypointIndex >= state.CurrentFollowingLane.Waypoints.Length - 1)
                    {
                        state.ExtendFollowingLane();
                        state.RemoveCurrentFollowingLane();
                        state.WaypointIndex = 1;
                    }
                    else
                    {
                        state.WaypointIndex++;
                    }

                    // Despawn if there are no lanes to follow.
                    if (state.FollowingLanes.Count == 0)
                        state.ShouldDespawn = true;
                }
            }
        }

        /// <summary>
        /// Read native data from <see cref="NpcVehicleInternalState"/>
        /// This job is sequential.
        /// </summary>
        struct ReadStateJob
        {
            // In
            public IReadOnlyList<NpcVehicleInternalState> States;

            // Out
            public NativeArray<NativeState> Results;

            /// <summary>
            /// Waypoints followed by the vehicle.
            /// Waypoints of all vehicles are merged into one native array.
            /// </summary>
            public NativeArray<Vector3> Waypoints;

            public void Execute()
            {
                for (var i = 0; i < States.Count; i++)
                {
                    if (States[i].Vehicle == null)
                        return;

                    var srcIndex = States[i].WaypointIndex;
                    var dstIndex = 0;
                    var offset = i * _maxWaypointCount;
                    foreach (var lane in States[i].FollowingLanes)
                    {
                        var length = Mathf.Min(
                            _maxWaypointCount - dstIndex,
                            lane.Waypoints.Length - srcIndex);

                        NativeArray<Vector3>.Copy(lane.Waypoints, srcIndex, Waypoints, offset + dstIndex, length);

                        dstIndex += length;

                        if (dstIndex >= _maxWaypointCount - 1)
                            break;

                        srcIndex = 1;
                    }

                    Results[i] = NativeState.Create(States[i], dstIndex);
                }
            }

        }

        /// <summary>
        /// Outputs <see cref="RaycastCommand"/> for checking ground existence.
        /// </summary>
        struct GroundCheckJob : IJobParallelFor
        {
            // In
            [ReadOnly] public NativeArray<NativeState> NativeStates;
            [WriteOnly] public NativeArray<RaycastCommand> Commands;

            public LayerMask GroundLayerMask;

            public void Execute(int index)
            {
                var from = NativeStates[index].FrontCenterPosition + Vector3.up * 2f;
                var direction = Vector3.down;
                var distance = 10f;

                var query = QueryParameters.Default;
                query.layerMask = GroundLayerMask;

                Commands[index] = new RaycastCommand(from, direction, query, distance);
            }
        }

        /// <summary>
        /// Outputs <see cref="BoxcastCommand"/> for checking front obstacles.
        /// </summary>
        struct ObstacleCheckJob : IJobParallelFor
        {
            public LayerMask ObstacleLayerMask;

            [ReadOnly] public NativeArray<NativeState> States;
            [ReadOnly] public NativeArray<Vector3> Waypoints;

            [WriteOnly] public NativeArray<BoxcastCommand> Commands;

            public void Execute(int index)
            {
                var stateIndex = index / _maxBoxcastCount;
                var waypointIndex = index % _maxBoxcastCount;
                var waypointOffset = stateIndex * _maxWaypointCount;

                if (waypointIndex >= States[stateIndex].WaypointCount)
                {
                    Commands[index] = new BoxcastCommand();
                    return;
                }

                var startPoint = waypointIndex == 0
                    ? States[stateIndex].FrontCenterPosition
                    : Waypoints[waypointOffset + waypointIndex - 1];

                startPoint += new Vector3(0, 1.5f, 0);

                // Reduce the detection range so that large sized vehicles can pass each other.
                //var boxCastExtents = States[stateIndex].Extents * 0.5f;
                var boxCastExtents = new Vector3(0.5f, 0.5f, 0.5f);
                boxCastExtents.y *= 1;
                boxCastExtents.z = 0.1f;
                var endPoint = Waypoints[waypointOffset + waypointIndex];

                endPoint += new Vector3(0, 1.5f, 0);

                var distance = Vector3.Distance(startPoint, endPoint);
                var direction = (endPoint - startPoint).normalized;
                var rotation = Quaternion.LookRotation(direction);
                Commands[index] = new BoxcastCommand(
                    startPoint,
                    boxCastExtents,
                    rotation,
                    direction,
                    new QueryParameters(ObstacleLayerMask),
                    distance
                );
            }
        }

        /// <summary>
        /// Calculate distance to the obstacle detected by boxcasts.
        /// </summary>
        struct CalculateObstacleDistanceJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<RaycastHit> HitInfoArray;
            [ReadOnly] public NativeArray<BoxcastCommand> Commands;
            [ReadOnly] public NativeArray<NativeState> NativeStates;

            public NativeArray<float> Distances;

            public void Execute(int index)
            {
                var hasHit = false;
                var totalDistance = 0f;
                var boxcastCount = Mathf.Min(_maxBoxcastCount, NativeStates[index].WaypointCount);
                for (var commandIndex = index * _maxBoxcastCount;
                     commandIndex < index * _maxBoxcastCount + boxcastCount;
                     commandIndex++)
                {
                    var hitInfo = HitInfoArray[commandIndex];
                    hasHit = hitInfo.distance != 0f || hitInfo.point != Vector3.zero;
                    if (hasHit)
                    {
                        totalDistance += hitInfo.distance;
                        break;
                    }
                    totalDistance += Commands[commandIndex].distance;
                }

                Distances[index] = hasHit
                    ? totalDistance
                    : float.MaxValue;
            }
        }

        /// <summary>
        /// Check sharp curve existence.
        /// </summary>
        struct CurveCheckJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<NativeState> NativeStates;
            [ReadOnly] public NativeArray<Vector3> Waypoints;

            [WriteOnly] public NativeArray<bool> IsTurnings;

            public void Execute(int index)
            {
                var state = NativeStates[index];
                var currentForward = Quaternion.AngleAxis(state.Yaw, Vector3.up) * Vector3.forward;
                var currentWaypointIndex = index * _maxWaypointCount;
                var elapsedDistance = Vector3.Distance(state.FrontCenterPosition, Waypoints[currentWaypointIndex]);
                var turnAngle = 0f;
                while (elapsedDistance < 40f)
                {
                    var currentWaypoint = Waypoints[currentWaypointIndex];
                    currentWaypointIndex++;

                    if (currentWaypointIndex >= index * _maxWaypointCount + state.WaypointCount)
                        break;

                    var nextWaypoint = Waypoints[currentWaypointIndex];
                    var nextForward = nextWaypoint - currentWaypoint;
                    elapsedDistance += Vector3.Distance(currentWaypoint, nextWaypoint);
                    turnAngle += Vector3.Angle(currentForward, nextForward);
                    currentForward = nextForward;
                }

                IsTurnings[index] = turnAngle > 45f;
            }
        }

        /// <summary>
        /// Check traffic conditions(EGOVehicle and NPCVehicle) on right of ways.
        /// This job is sequential.
        /// </summary>
        struct RightOfWayCheckJob
        {
            public static float MinimumDistanceToIntersection = 18f;

            public static float MaximumOverrunStopPointForLaneRules = 1f;

            public static float DifferenceOrientationDegreesImplyingPerpendicularRoad = 35f;

            public static float MinimumDistanceBetweenNPCs = 70f;

            // In
            public Transform EgoTransform;

            // In/Out
            public IReadOnlyList<NpcVehicleInternalState> States;

            /// <summary>
            /// Checks traffic conditions on right of ways.
            /// This method consists of the following steps:<br/>
            /// - Find
            /// </summary>
            public void Execute()
            {
                foreach (var refState in States)
                {
                    if (refState.ShouldDespawn)
                        continue;

                    switch (refState.YieldPhase)
                    {
                        case NpcVehicleYieldPhase.None:
                            refState.YieldLane = null;
                            if (isEnteringIntersection(refState))
                            {
                                refState.YieldPoint = refState.FirstLaneWithIntersection.GetStopPoint();
                                refState.YieldPhase = NpcVehicleYieldPhase.EnteringIntersection;
                            }
                            else if (refState.isOnIntersection)
                            {
                                refState.YieldPhase = NpcVehicleYieldPhase.AtIntersection;
                            }

                            if (refState.YieldPhase != NpcVehicleYieldPhase.None)
                                if (refState.isIntersectionWithYieldingLane)
                                    refState.YieldLane = refState.FirstLaneWithIntersection;
                                else
                                    refState.YieldLane = null;
                            break;

                        case NpcVehicleYieldPhase.EnteringIntersection:
                            if (refState.isOnIntersection)
                            {
                                refState.YieldPhase = NpcVehicleYieldPhase.AtIntersection;
                            }
                            else if (ShouldYieldDueToLanes(refState, States, false))
                            {
                                refState.YieldPhase = NpcVehicleYieldPhase.LanesRulesEnteringIntersection;
                            }
                            else if (isIntersectionBusy(refState, States))
                            {
                                refState.YieldPhase = NpcVehicleYieldPhase.IntersectionBlocked;
                            }
                            else if (isLeftHandRuleEnteringIntersection(refState, States))
                            {
                                refState.YieldPhase = NpcVehicleYieldPhase.LeftHandRuleEnteringIntersection;
                            }
                            break;

                        case NpcVehicleYieldPhase.AtIntersection:
                            if (!refState.isOnIntersection)
                            {
                                refState.YieldPhase = NpcVehicleYieldPhase.None;
                            }
                            else if (ShouldYieldDueToLanes(refState, States, true))
                            {
                                refState.YieldPhase = NpcVehicleYieldPhase.LanesRulesAtIntersection;
                            }
                            else if (isSomeVehicleForcingPriority(refState, States))
                            {
                                refState.YieldPhase = NpcVehicleYieldPhase.ForcingPriority;
                            }
                            else if (isLeftHandRuleOnIntersection(refState, States))
                            {
                                refState.YieldPhase = NpcVehicleYieldPhase.LeftHandRuleAtIntersection;
                            }
                            break;

                        case NpcVehicleYieldPhase.IntersectionBlocked:
                            if (!isIntersectionBusy(refState, States))
                            {
                                refState.YieldPhase = NpcVehicleYieldPhase.None;
                            }
                            break;
                        case NpcVehicleYieldPhase.LanesRulesEnteringIntersection:
                            if (refState.isOnIntersection || !ShouldYieldDueToLanes(refState, States, false))
                            {
                                refState.YieldPhase = NpcVehicleYieldPhase.None;
                            }
                            break;
                        case NpcVehicleYieldPhase.LanesRulesAtIntersection:
                            if (!ShouldYieldDueToLanes(refState, States, true))
                            {
                                refState.YieldPhase = NpcVehicleYieldPhase.None;
                            }
                            break;
                        case NpcVehicleYieldPhase.LeftHandRuleEnteringIntersection:
                            if (refState.isOnIntersection || !isLeftHandRuleEnteringIntersection(refState, States))
                            {
                                refState.YieldPhase = NpcVehicleYieldPhase.None;
                            }
                            break;
                        case NpcVehicleYieldPhase.LeftHandRuleAtIntersection:
                            if (!isLeftHandRuleOnIntersection(refState, States))
                            {
                                refState.YieldPhase = NpcVehicleYieldPhase.None;
                            }
                            break;
                        case NpcVehicleYieldPhase.ForcingPriority:
                            if (!isSomeVehicleForcingPriority(refState, States))
                            {
                                refState.YieldPhase = NpcVehicleYieldPhase.None;
                            }
                            break;
                        default:
                            throw new Exception($"Unattended NPCVehicleYieldPhase case: {refState.YieldPhase}");
                    }
                }
            }

            static private bool isCloseEachOther(NpcVehicleInternalState refState, NpcVehicleInternalState otherState)
            {
                return LaneletGeometryUtility.Distance2D(refState.Vehicle.transform.position, otherState.Vehicle.transform.position) < MinimumDistanceBetweenNPCs;
            }

            static private bool isYieldingDueToRules(NpcVehicleInternalState refState)
            {
                return refState.YieldPhase == NpcVehicleYieldPhase.LeftHandRuleEnteringIntersection ||
                refState.YieldPhase == NpcVehicleYieldPhase.LeftHandRuleAtIntersection ||
                refState.YieldPhase == NpcVehicleYieldPhase.LanesRulesEnteringIntersection ||
                refState.YieldPhase == NpcVehicleYieldPhase.LanesRulesAtIntersection;
            }

            static private bool isYieldingDueToLanesBeforeIntersection(NpcVehicleInternalState refState)
            {
                return
                refState.YieldPhase == NpcVehicleYieldPhase.LeftHandRuleEnteringIntersection ||
                refState.YieldPhase == NpcVehicleYieldPhase.LanesRulesEnteringIntersection;
            }

            static private bool theSameIntersectionLane(NpcVehicleInternalState refState, NpcVehicleInternalState otherState)
            {
                return refState.FirstLaneWithIntersection?.name == otherState.FirstLaneWithIntersection?.name;
            }

            static private bool isEnteringFromTheSameSide(NpcVehicleInternalState refState, NpcVehicleInternalState otherState)
            {
                if (otherState.FirstLaneWithIntersection == null || refState.FirstLaneWithIntersection == null)
                    return false;
                else
                    return Vector3.Distance(refState.FirstIntersectionWaypoint.Value, otherState.FirstIntersectionWaypoint.Value) < 3f;
            }

            static private bool isVehicleOnTheLeft(NpcVehicleInternalState refState, NpcVehicleInternalState otherState)
            {
                Vector3 refPosition = new Vector3(refState.FrontCenterPosition.x, 0f, refState.FrontCenterPosition.z);
                Vector3 positionBack = new Vector3(otherState.BackCenterPosition.x, 0f, otherState.BackCenterPosition.z);
                Vector3 positionFront = new Vector3(otherState.FrontCenterPosition.x, 0f, otherState.FrontCenterPosition.z);
                var crossFront = Vector3.Cross(refState.Forward, positionFront - refPosition).y;
                var crossBack = Vector3.Cross(refState.Forward, positionBack - refPosition).y;
                return crossFront < 0f || crossBack < 0f;
            }


            /// <summary>
            /// Check if the otherState must yield priority based on RightOfWayLanes to refState at the intersection
            /// </summary>
            static private bool shouldHavePriority(NpcVehicleInternalState refState, NpcVehicleInternalState otherState)
            {
                if (otherState.FirstLaneWithIntersection != null && refState.FirstLaneWithIntersection != null)
                {
                    foreach (TrafficLane otherStateLane in otherState.FirstLaneWithIntersection.RightOfWayLanes)
                    {
                        if (otherStateLane.name == refState.FirstLaneWithIntersection.name)
                        {
                            return true;
                        }
                    }
                }
                return false;
            }

            /// <summary>
            /// Check if the refState must consider the otherState when examining yielding priority
            /// </summary>
            static private bool shouldBeConsideredForYielding(NpcVehicleInternalState refState, NpcVehicleInternalState otherState)
            {
                return refState.Vehicle.Id != otherState.Vehicle.Id && isEnteringIntersection(otherState);
            }

            /// <summary>
            /// Check if the refState should be consider when examining yielding priority
            /// </summary>
            static private bool isEnteringIntersection(NpcVehicleInternalState refState)
            {
                return refState.DistanceToIntersection <= MinimumDistanceToIntersection && !refState.ObstructedByVehicleBehindIntersection;
            }

            static private bool isLeftHandRuleEnteringIntersection(NpcVehicleInternalState refState, IReadOnlyList<NpcVehicleInternalState> states)
            {
                foreach (var otherState in states)
                {
                    if (!isCloseEachOther(refState, otherState))
                        continue;

                    if (!shouldBeConsideredForYielding(refState, otherState))
                        continue;

                    if (isYieldingDueToLanesBeforeIntersection(otherState) || otherState.yieldingPriorityAtTrafficLight)
                        continue;

                    if (isEnteringFromTheSameSide(refState, otherState))
                        continue;

                    if (!isVehicleOnTheLeft(refState, otherState))
                        continue;

                    if (intersectOverall(refState, otherState))
                    {
                        refState.DominatingVehicle = otherState.Vehicle.NpcVehicle.RigidBodyTransform;
                        refState.YieldPoint = refState.FollowingLanes[0].GetStopPoint();
                        return true;
                    }
                }
                return false;
            }

            static private bool isLeftHandRuleOnIntersection(NpcVehicleInternalState refState, IReadOnlyList<NpcVehicleInternalState> states)
            {
                foreach (var otherState in states)
                {
                    if (!isCloseEachOther(refState, otherState))
                        continue;

                    if (!shouldBeConsideredForYielding(refState, otherState))
                        continue;

                    if (!otherState.isOnIntersection)
                        continue;

                    if (isYieldingDueToRules(otherState))
                        continue;

                    if (isEnteringFromTheSameSide(refState, otherState))
                        continue;

                    if (!isVehicleOnTheLeft(refState, otherState))
                        continue;

                    // if the refState should yield priority due to lane rules
                    // ignore this condition and allow the refState to proceed to LANES_RULES_**
                    if (shouldHavePriority(otherState, refState))
                        continue;

                    if (intersectNowFront(refState, otherState))
                    {
                        refState.DominatingVehicle = otherState.Vehicle.NpcVehicle.RigidBodyTransform;
                        refState.YieldPoint = refState.FrontCenterPosition;
                        return true;
                    }
                }
                return false;
            }

            static private bool isIntersectionBusy(NpcVehicleInternalState refState, IReadOnlyList<NpcVehicleInternalState> states)
            {
                foreach (var otherState in states)
                {
                    if (!isCloseEachOther(refState, otherState))
                        continue;

                    if (!shouldBeConsideredForYielding(refState, otherState))
                        continue;

                    if (!otherState.isOnIntersection)
                        continue;

                    if (isYieldingDueToRules(otherState) && isOnPerpendicularRoad(refState, otherState))
                        continue;

                    if (isEnteringFromTheSameSide(refState, otherState))
                        continue;

                    if (intersectOverall(refState, otherState))
                    {
                        refState.DominatingVehicle = otherState.Vehicle.NpcVehicle.RigidBodyTransform;
                        refState.YieldPoint = refState.FollowingLanes[0].GetStopPoint();
                        return true;
                    }
                }
                return false;

                static bool isOnPerpendicularRoad(NpcVehicleInternalState refState, NpcVehicleInternalState otherState)
                {
                    var diffAngleDegrees = Mathf.Abs(refState.Yaw - otherState.Yaw);
                    return (diffAngleDegrees < DifferenceOrientationDegreesImplyingPerpendicularRoad ||
                        diffAngleDegrees > 180f - DifferenceOrientationDegreesImplyingPerpendicularRoad);
                }
            }

            static private bool isSomeVehicleForcingPriority(NpcVehicleInternalState refState, IReadOnlyList<NpcVehicleInternalState> states)
            {
                foreach (var otherState in states)
                {
                    if (!isCloseEachOther(refState, otherState))
                        continue;

                    if (!shouldBeConsideredForYielding(refState, otherState))
                        continue;

                    if (isYieldingDueToRules(otherState))
                        continue;

                    if (!shouldHavePriority(refState, otherState))
                        continue;

                    if (intersectNowFront(refState, otherState))
                    {
                        refState.YieldPoint = refState.FrontCenterPosition;
                        refState.DominatingVehicle = otherState.Vehicle.NpcVehicle.RigidBodyTransform;
                        return true;
                    }
                }
                return false;
            }


            private bool ShouldYieldDueToLanes(NpcVehicleInternalState refState, IReadOnlyList<NpcVehicleInternalState> states, bool refOnIntersection)
            {
                if (refState.YieldLane == null)
                    return false;

                var isNowYielding = isYieldingDueToRules(refState);
                var stopPoint = refOnIntersection ? (refState.YieldLane.GetStopPoint(1)) : (refState.YieldLane.GetStopPoint());
                // provide hysteresis equal to maximumOverrunStopPointForLaneRules/2.0
                if (isNowYielding && refState.SignedDistanceToPointOnLane(refState.YieldPoint) < -MaximumOverrunStopPointForLaneRules)
                    return false;
                else if (!isNowYielding && refState.SignedDistanceToPointOnLane(stopPoint) < -MaximumOverrunStopPointForLaneRules / 2.0)
                    return false;

                Transform dominatingVehicle = null;
                foreach (var lane in refState.YieldLane.RightOfWayLanes)
                {
                    if (IsLaneDominatedByAny(lane, states, refState, refOnIntersection, out dominatingVehicle))
                    {
                        refState.YieldPoint = stopPoint;
                        refState.DominatingVehicle = dominatingVehicle;
                        return true;
                    }
                    else if (IsLaneDominatedByVehicle(lane, EgoTransform.position, EgoTransform.forward))
                    {
                        refState.YieldPoint = stopPoint;
                        refState.DominatingVehicle = EgoTransform;
                        return true;
                    }
                }
                return false;


                static bool IsLaneDominatedByAny(TrafficLane lane, IReadOnlyList<NpcVehicleInternalState> states,
                    NpcVehicleInternalState refState, bool refOnIntersection, out Transform dominatingVehicle)
                {
                    dominatingVehicle = null;
                    foreach (var otherState in states)
                    {
                        if (!isCloseEachOther(refState, otherState))
                            continue;

                        if (!shouldBeConsideredForYielding(refState, otherState))
                            continue;

                        if (otherState.yieldingPriorityAtTrafficLight)
                            continue;

                        if (refOnIntersection && (isYieldingDueToRules(otherState) || otherState.YieldPhase == NpcVehicleYieldPhase.IntersectionBlocked))
                            continue;

                        if (!IsLaneDominatedBy(lane, otherState))
                            continue;

                        if (!intersectOverall(refState, otherState))
                            continue;

                        dominatingVehicle = otherState.Vehicle.NpcVehicle.RigidBodyTransform;
                        return true;
                    }
                    return false;
                }

                static bool IsLaneDominatedBy(TrafficLane lane, NpcVehicleInternalState state)
                {
                    return (state.CurrentFollowingLane._intersectionLane || state.IsNextLaneIntersection) && state.FirstLaneWithIntersection == lane;
                }

                /// Check if <paramref name="lane"/> is dominated by a vehicle whose position is <paramref name="vehiclePosition"/> and forward direction is <paramref name="vehicleForward"/>.
                /// All vehicles trying to pass on a non-priority lane needs to yield if any vehicle dominates the right of way lane.<br/>
                /// It is implemented for vehicles for which lane information is not explicitly maintained, such as EGO vehicles.
                /// </summary>
                static bool IsLaneDominatedByVehicle(TrafficLane lane, Vector3 vehiclePosition, Vector3 vehicleForward)
                {
                    vehiclePosition.y = 0f;
                    vehicleForward.y = 0f;
                    for (var i = 0; i < lane.Waypoints.Length - 1; i++)
                    {
                        var wp0 = lane.Waypoints[i];
                        var wp1 = lane.Waypoints[i + 1];
                        wp0.y = 0f;
                        wp1.y = 0f;
                        if (!IsInLaneSection(wp0, wp1, vehiclePosition))
                            continue;
                        if (Vector3.Angle(wp1 - wp0, vehicleForward) < 30f)
                            return true;
                    }
                    return false;

                    static bool IsInLaneSection(Vector3 v, Vector3 w, Vector3 p, float extent = 2f)
                    {
                        var l2 = (w - v).sqrMagnitude;
                        if (l2 == 0f)
                            return false;
                        var t = Vector3.Dot(p - v, w - v) / l2;
                        if (t > 1f || t < 0f)
                            return false;
                        var projection = v + t * (w - v);
                        return Vector3.Distance(p, projection) <= extent;
                    }
                }
            }


            static private bool CheckIfLinesIntersect(Vector3 A1, Vector3 B1, Vector3 A2, Vector3 B2)
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
                        return false;
                }
                else if (alphaNumerator > 0 || alphaNumerator < denominator || betaNumerator > 0 || betaNumerator < denominator)
                {
                    return false;
                }
                return true;
            }

            static private bool intersectOverall(NpcVehicleInternalState refState, NpcVehicleInternalState otherState)
            {
                Vector3? refStartNullable = refState.FirstIntersectionWaypoint;
                Vector3? otherStartNullable = otherState.FirstIntersectionWaypoint;
                if (refStartNullable == null || otherStartNullable == null)
                    return false;
                Vector3? refGoalNullable = refState.LastIntersectionWaypoint;
                Vector3? otherGoalNullable = otherState.LastIntersectionWaypoint;
                if (refGoalNullable == null || otherGoalNullable == null)
                    return false;

                if (!CheckIfLinesIntersect(refStartNullable.Value, refGoalNullable.Value, otherStartNullable.Value, otherGoalNullable.Value))
                    return false;

                var refLane = refState.FirstLaneWithIntersection;
                var otherLane = otherState.FirstLaneWithIntersection;

                var refFirstWaypointIndex = (refLane.name == refState.CurrentFollowingLane.name) ? refState.WaypointIndex : 0;
                var otherFirstWaypointIndex = (otherLane.name == otherState.CurrentFollowingLane.name) ? otherState.WaypointIndex : 0;
                // check the intersection between all waypoints of these lanes (starting from CurrentWaypoint)
                for (int i = refFirstWaypointIndex; i < refLane.Waypoints.Length - 1; i++)
                {
                    for (int j = otherFirstWaypointIndex; j < otherLane.Waypoints.Length - 1; j++)
                    {
                        if (Vector3.Distance(refLane.Waypoints[i + 1], otherLane.Waypoints[j + 1]) < 1f)
                            return true;

                        if (CheckIfLinesIntersect(refLane.Waypoints[i], refLane.Waypoints[i + 1], otherLane.Waypoints[j], otherLane.Waypoints[j + 1]))
                            return true;
                    }
                }

                if (refLane.name == refState.CurrentFollowingLane.name)
                {
                    // Check short section [BackCenterPosition->CurrentWaypoint] with all sections from otherLane
                    for (int j = otherFirstWaypointIndex; j < otherLane.Waypoints.Length - 1; j++)
                    {
                        if (CheckIfLinesIntersect(refState.BackCenterPosition, refState.CurrentWaypoint, otherLane.Waypoints[j], otherLane.Waypoints[j + 1]))
                            return true;
                    }
                }


                if (otherLane.name == otherState.CurrentFollowingLane.name)
                {
                    // Check short section [otherState.BackCenterPosition->otherState.CurrentWaypoint] with all sections from otherLane
                    for (int i = refFirstWaypointIndex; i < refLane.Waypoints.Length - 1; i++)
                    {
                        if (CheckIfLinesIntersect(refLane.Waypoints[i], refLane.Waypoints[i + 1], otherState.BackCenterPosition, otherState.CurrentWaypoint))
                            return true;
                    }
                }

                // Check short sections [BackCenterPosition->CurrentWaypoint]x[otherState.BackCenterPosition->otherState.CurrentWaypoint]
                return CheckIfLinesIntersect(refState.BackCenterPosition, refState.CurrentWaypoint, otherState.BackCenterPosition, otherState.CurrentWaypoint);
            }

            static private bool intersectNowFront(NpcVehicleInternalState refState, NpcVehicleInternalState otherState)
            {
                if (!refState.CurrentFollowingLane._intersectionLane || !otherState.CurrentFollowingLane._intersectionLane)
                    return false;

                var refLane = refState.CurrentFollowingLane;
                var otherLane = otherState.CurrentFollowingLane;
                // check the intersection between all waypoints of current lanes (starting from CurrentWaypoint)
                for (int i = refState.WaypointIndex; i < refState.CurrentFollowingLane.Waypoints.Length - 1; i++)
                {
                    for (int j = otherState.WaypointIndex; j < otherState.CurrentFollowingLane.Waypoints.Length - 1; j++)
                    {
                        if (Vector3.Distance(refLane.Waypoints[i + 1], otherLane.Waypoints[j + 1]) < 1f)
                            return true;

                        if (CheckIfLinesIntersect(refLane.Waypoints[i], refLane.Waypoints[i + 1], otherLane.Waypoints[j], otherLane.Waypoints[j + 1]))
                            return true;
                    }
                }

                Vector3 refPose = refState.FrontCenterPosition;
                Vector3 refNearestWaypoint = refState.CurrentWaypoint;
                // Check short section [FrontCenterPosition->CurrentWaypoint] with all sections from otherLane
                for (int j = otherState.WaypointIndex; j < otherState.CurrentFollowingLane.Waypoints.Length - 1; j++)
                {
                    if (Vector3.Distance(refNearestWaypoint, otherLane.Waypoints[j + 1]) < 1f)
                        return true;

                    if (CheckIfLinesIntersect(refPose, refNearestWaypoint, otherLane.Waypoints[j], otherLane.Waypoints[j + 1]))
                        return true;
                }

                //BackCenterPosition is extended by the Width due to the use of lines intersection analysis (not bounding-boxes)
                //vehicles have their own width - this can cause collisions at perpendicular intersections
                Vector3 otherPose = otherState.ExpandedBackCenterPosition(refState.Width);
                Vector3 otherNearestWaypoint = otherState.CurrentWaypoint;
                // Check short section [otherState.ExpandedBackCenterPosition->otherState.CurrentWaypoint] with all sections from otherLane
                for (int i = refState.WaypointIndex; i < refState.CurrentFollowingLane.Waypoints.Length - 1; i++)
                {
                    if (Vector3.Distance(refLane.Waypoints[i + 1], otherNearestWaypoint) < 1f)
                        return true;

                    if (CheckIfLinesIntersect(refLane.Waypoints[i], refLane.Waypoints[i + 1], otherPose, otherNearestWaypoint))
                        return true;
                }

                // Check short sections [BackCenterPosition->CurrentWaypoint]x[[otherState.ExpandedBackCenterPosition->otherState.CurrentWaypoint]
                return CheckIfLinesIntersect(refPose, refNearestWaypoint, otherPose, otherNearestWaypoint);
            }
        }

        /// <summary>
        /// Check traffic light states.
        /// This job is sequential.
        /// </summary>
        struct TrafficLightCheckJob
        {
            // In/Out
            public IReadOnlyList<NpcVehicleInternalState> States;

            public void Execute()
            {
                foreach (var state in States)
                {
                    if (state.ShouldDespawn)
                        continue;

                    // Find next stop line with a traffic light.
                    // If the vehicle is stopping, use cached stop line.
                    // This is in case a vehicle crosses the stop line while stopping.
                    if (state.TrafficLightPassability != TrafficLightPassability.Red)
                    {
                        // Find far traffic light by extending lane to follow.
                        // This is done because if a lane is too short, it may not be able to stop at a stop line.
                        if (state.FollowingLanes.Count < 3)
                        {
                            state.ExtendFollowingLane();
                        }

                        state.TrafficLightLane = null;
                        foreach (var lane in state.FollowingLanes)
                        {
                            if (lane.StopLine?.TrafficLight == null)
                                continue;

                            state.TrafficLightLane = lane;
                            break;
                        }

                    }

                    var trafficLight = state.TrafficLightLane?.StopLine.TrafficLight;
                    if (trafficLight == null)
                    {
                        state.TrafficLightPassability = TrafficLightPassability.Green;
                        continue;
                    }

                    var intersectionLane = state.GetOrExtendNextLane(state.TrafficLightLane);
                    if (intersectionLane == null)
                        continue;
                    state.TrafficLightPassability =
                        RandomTrafficUtils.GetPassability(trafficLight, intersectionLane.TurnDirection);
                }
            }
        }

        /// <summary>
        /// Write native data to <see cref="NpcVehicleInternalState"/>.
        /// This job is sequential.
        /// </summary>
        struct WriteStateJob
        {
            [ReadOnly] public NativeArray<RaycastHit> GroundHitInfoArray;
            [ReadOnly] public NativeArray<float> ObstacleDistances;
            [ReadOnly] public NativeArray<bool> IsTurnings;

            public IReadOnlyList<NpcVehicleInternalState> States;

            public void Execute()
            {
                for (var i = 0; i < States.Count; i++)
                {
                    if (GroundHitInfoArray[i].collider == null)
                        States[i].ShouldDespawn = true;

                    States[i].DistanceToFrontVehicle = ObstacleDistances[i];
                    States[i].IsTurning = IsTurnings[i];
                }
            }
        }

        /// <summary>
        /// Maximum number of waypoints held per vehicle.
        /// Calculation cost is proportional to this value.
        /// </summary>
        const int _maxWaypointCount = 10;

        /// <summary>
        /// Maximum number of boxcasts executed per vehicle.
        /// Calculation cost is proportional to this value.
        /// This value should be less than or equal to <see cref="_maxWaypointCount"/>.
        /// </summary>
        const int _maxBoxcastCount = 5;

        readonly LayerMask _obstacleLayerMask;
        readonly LayerMask _groundLayerMask;

        // Native states
        NativeArray<NativeState> _nativeStates;
        NativeArray<Vector3> _waypoints;

        // Ground Check
        NativeArray<RaycastCommand> _raycastCommands;
        NativeArray<RaycastHit> _groundHitInfoArray;

        // Obstacle Check
        NativeArray<BoxcastCommand> _boxcastCommands;
        NativeArray<RaycastHit> _obstacleHitInfoArray;
        NativeArray<float> _obstacleDistances;

        // Curve Check
        NativeArray<bool> _isTurnings;

        // Job Handles
        JobHandle _groundCheckJobHandle;
        JobHandle _raycastJobHandle;
        JobHandle _obstacleCheckJobHandle;
        JobHandle _boxcastJobHandle;
        JobHandle _calculateObstacleDistanceJobHandle;
        JobHandle _curveCheckJobHandle;

        public NpcVehicleCognitionStep(LayerMask obstacleLayerMask, LayerMask groundLayerMask, int maxVehicleCount)
        {
            this._obstacleLayerMask = obstacleLayerMask;
            this._groundLayerMask = groundLayerMask;

            _nativeStates = new NativeArray<NativeState>(maxVehicleCount, Allocator.Persistent);
            _waypoints = new NativeArray<Vector3>(maxVehicleCount * _maxWaypointCount, Allocator.Persistent);

            _raycastCommands = new NativeArray<RaycastCommand>(maxVehicleCount, Allocator.Persistent);
            _groundHitInfoArray = new NativeArray<RaycastHit>(maxVehicleCount, Allocator.Persistent);

            _boxcastCommands = new NativeArray<BoxcastCommand>(maxVehicleCount * _maxBoxcastCount, Allocator.Persistent);
            _obstacleHitInfoArray =
                new NativeArray<RaycastHit>(maxVehicleCount * _maxBoxcastCount, Allocator.Persistent);
            _obstacleDistances = new NativeArray<float>(maxVehicleCount, Allocator.Persistent);

            _isTurnings = new NativeArray<bool>(maxVehicleCount, Allocator.Persistent);
        }

        public void Dispose()
        {
            var dependsOn = JobHandle.CombineDependencies(
                _raycastJobHandle, _calculateObstacleDistanceJobHandle, _curveCheckJobHandle);

            _nativeStates.Dispose(dependsOn);
            _waypoints.Dispose(dependsOn);

            _raycastCommands.Dispose(dependsOn);
            _groundHitInfoArray.Dispose(dependsOn);

            _boxcastCommands.Dispose(dependsOn);
            _obstacleHitInfoArray.Dispose(dependsOn);
            _obstacleDistances.Dispose(dependsOn);

            _isTurnings.Dispose(dependsOn);
        }

        public void Execute(
            IReadOnlyList<NpcVehicleInternalState> states,
            Transform egoTransform)
        {
            Profiler.BeginSample("Cognition.CheckNextWaypoint");

            new NextWaypointCheckJob
            {
                States = states
            }.Execute();

            Profiler.EndSample();
            Profiler.BeginSample("Cognition.ReadState");

            new ReadStateJob
            {
                Results = _nativeStates,
                Waypoints = _waypoints,
                States = states
            }.Execute();

            Profiler.EndSample();

            Profiler.BeginSample("Cognition.JobSchedule");

            _obstacleCheckJobHandle =
                new ObstacleCheckJob
                {
                    Commands = _boxcastCommands,
                    States = _nativeStates,
                    ObstacleLayerMask = _obstacleLayerMask,
                    Waypoints = _waypoints
                }.Schedule(_boxcastCommands.Length, 16);

            _boxcastJobHandle =
                BoxcastCommand.ScheduleBatch(
                    _boxcastCommands,
                    _obstacleHitInfoArray,
                    16,
                    _obstacleCheckJobHandle);

            // Start background jobs
            JobHandle.ScheduleBatchedJobs();

            _groundCheckJobHandle =
                new GroundCheckJob
                {
                    Commands = _raycastCommands,
                    GroundLayerMask = _groundLayerMask,
                    NativeStates = _nativeStates
                }.Schedule(_nativeStates.Length, 8);

            _raycastJobHandle =
                RaycastCommand.ScheduleBatch(
                    _raycastCommands,
                    _groundHitInfoArray,
                    8,
                    _groundCheckJobHandle);

            _calculateObstacleDistanceJobHandle = new CalculateObstacleDistanceJob
            {
                HitInfoArray = _obstacleHitInfoArray,
                Commands = _boxcastCommands,
                NativeStates = _nativeStates,
                Distances = _obstacleDistances
            }.Schedule(_nativeStates.Length, 8, _boxcastJobHandle);

            _curveCheckJobHandle = new CurveCheckJob
            {
                NativeStates = _nativeStates,
                Waypoints = _waypoints,
                IsTurnings = _isTurnings
            }.Schedule(_nativeStates.Length, 8);

            // Start rest background jobs
            JobHandle.ScheduleBatchedJobs();

            Profiler.EndSample();
            Profiler.BeginSample("Cognition.CheckRightOfWay");

            new RightOfWayCheckJob
            {
                EgoTransform = egoTransform,
                States = states
            }.Execute();

            Profiler.EndSample();
            Profiler.BeginSample("Cognition.CheckTrafficLight");

            new TrafficLightCheckJob
            {
                States = states
            }.Execute();

            Profiler.EndSample();
            Profiler.BeginSample("Cognition.WaitJobs");

            _obstacleCheckJobHandle.Complete();
            _groundCheckJobHandle.Complete();
            _boxcastJobHandle.Complete();
            _raycastJobHandle.Complete();
            _calculateObstacleDistanceJobHandle.Complete();
            _curveCheckJobHandle.Complete();

            Profiler.EndSample();
            Profiler.BeginSample("Cognition.WriteState");

            new WriteStateJob
            {
                GroundHitInfoArray = _groundHitInfoArray,
                ObstacleDistances = _obstacleDistances,
                IsTurnings = _isTurnings,
                States = states
            }.Execute();

            Profiler.EndSample();
        }

        public void ShowGizmos(IReadOnlyList<NpcVehicleInternalState> states, bool showYieldingPhase, bool showObstacleChecking)
        {
            if (showYieldingPhase)
            {
                foreach (var state in states)
                {
                    var stateCurrentPosition = state.FrontCenterPosition;
                    stateCurrentPosition.y = state.Vehicle.transform.position.y + 2f;

                    if (state.YieldPhase == NpcVehicleYieldPhase.None ||
                        state.YieldPhase == NpcVehicleYieldPhase.EnteringIntersection ||
                        state.YieldPhase == NpcVehicleYieldPhase.AtIntersection)
                    {
                        continue;
                    }

                    switch (state.YieldPhase)
                    {
                        case NpcVehicleYieldPhase.IntersectionBlocked:
                            Gizmos.color = Color.blue;
                            break;
                        case NpcVehicleYieldPhase.LeftHandRuleEnteringIntersection:
                            Gizmos.color = Color.gray;
                            break;
                        case NpcVehicleYieldPhase.LeftHandRuleAtIntersection:
                            Gizmos.color = Color.black;
                            break;
                        case NpcVehicleYieldPhase.LanesRulesEnteringIntersection:
                            Gizmos.color = Color.yellow;
                            break;
                        case NpcVehicleYieldPhase.LanesRulesAtIntersection:
                            Gizmos.color = Color.red;
                            break;
                        case NpcVehicleYieldPhase.ForcingPriority:
                            Gizmos.color = Color.magenta;
                            break;
                    }
                    Gizmos.DrawCube(state.YieldPoint, new Vector3(1.0f, 0.2f, 1.0f));
                    Gizmos.DrawSphere(stateCurrentPosition, 0.5f);
                    if (state.DominatingVehicle != null)
                        Gizmos.DrawLine(stateCurrentPosition, state.DominatingVehicle.transform.position);
                }
            }

            // Obstacle checking
            if (showObstacleChecking)
            {
                for (var stateIndex = 0; stateIndex < states.Count; stateIndex++)
                {
                    Gizmos.color = states[stateIndex].IsStoppedByFrontVehicle
                        ? new Color(1f, 0.2f, 0f)
                        : Color.cyan;

                    var boxcastCount = Mathf.Min(_maxBoxcastCount, _nativeStates[stateIndex].WaypointCount);
                    for (var commandIndex = stateIndex * _maxBoxcastCount;
                            commandIndex < stateIndex * _maxBoxcastCount + boxcastCount;
                            commandIndex++)
                    {
                        var hitInfo = _obstacleHitInfoArray[commandIndex];
                        var hasHit = hitInfo.collider != null;

                        var command = _boxcastCommands[commandIndex];
                        var startPoint = command.center;
                        var direction = command.direction;
                        var distance = hasHit
                            ? hitInfo.distance
                            : command.distance;
                        var extents = command.halfExtents;
                        var destination = startPoint + distance * direction;
                        var rotation = Quaternion.LookRotation(direction);
                        Gizmos.matrix = Matrix4x4.TRS((destination + startPoint) / 2f, rotation, Vector3.one);
                        var cubeSize = extents * 2f;
                        cubeSize.z = distance;
                        Gizmos.DrawWireCube(Vector3.zero, cubeSize);
                        Gizmos.matrix = Matrix4x4.identity;

                        if (hasHit)
                            break;
                    }
                }
            }
        }
    }
}
