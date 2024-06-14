using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using UnityEngine.Profiling;

namespace AWSIM.TrafficSimulation
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
    public class NPCVehicleCognitionStep : IDisposable
    {
        /// <summary>
        /// NativeContainer compatible data of NPC vehicles for JobSystem input.
        /// </summary>
        private struct NativeState
        {
            public Vector3 Extents;
            public Vector3 FrontCenterPosition;
            public float Yaw;
            public int WaypointCount;

            public static NativeState Create(NPCVehicleInternalState state, int waypointCount)
            {
                return new NativeState
                {
                    Extents = state.Vehicle.Bounds.extents,
                    FrontCenterPosition = state.FrontCenterPosition,
                    Yaw = state.Yaw,
                    WaypointCount = waypointCount
                };
            }
        }

        /// <summary>
        /// Check next lane and waypoint to follow.
        /// </summary>
        private struct NextWaypointCheckJob
        {
            // In/Out
            public IReadOnlyList<NPCVehicleInternalState> States;

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
        /// Read native data from <see cref="NPCVehicleInternalState"/>
        /// This job is sequential.
        /// </summary>
        private struct ReadStateJob
        {
            // In
            public IReadOnlyList<NPCVehicleInternalState> States;

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
                    var offset = i * MaxWaypointCount;
                    foreach (var lane in States[i].FollowingLanes)
                    {
                        var length = Mathf.Min(
                            MaxWaypointCount - dstIndex,
                            lane.Waypoints.Length - srcIndex);

                        NativeArray<Vector3>.Copy(lane.Waypoints, srcIndex, Waypoints, offset + dstIndex, length);

                        dstIndex += length;

                        if (dstIndex >= MaxWaypointCount - 1)
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
        private struct GroundCheckJob : IJobParallelFor
        {
            // In
            [ReadOnly] public NativeArray<NativeState> NativeStates;
            [WriteOnly] public NativeArray<RaycastCommand> Commands;

            public LayerMask GroundLayerMask;

            public void Execute(int index)
            {
                Commands[index] = new RaycastCommand
                {
                    from = NativeStates[index].FrontCenterPosition + Vector3.up * 2f,
                    direction = Vector3.down,
                    distance = 10f,
                    layerMask = GroundLayerMask,
                    maxHits = 1
                };
            }
        }

        /// <summary>
        /// Outputs <see cref="BoxcastCommand"/> for checking front obstacles.
        /// </summary>
        private struct ObstacleCheckJob : IJobParallelFor
        {
            public LayerMask VehicleLayerMask;

            [ReadOnly] public NativeArray<NativeState> States;
            [ReadOnly] public NativeArray<Vector3> Waypoints;

            [WriteOnly] public NativeArray<BoxcastCommand> Commands;

            public void Execute(int index)
            {
                var stateIndex = index / MaxBoxcastCount;
                var waypointIndex = index % MaxBoxcastCount;
                var waypointOffset = stateIndex * MaxWaypointCount;

                if (waypointIndex >= States[stateIndex].WaypointCount)
                {
                    Commands[index] = new BoxcastCommand();
                    return;
                }

                var startPoint = waypointIndex == 0
                    ? States[stateIndex].FrontCenterPosition
                    : Waypoints[waypointOffset + waypointIndex - 1];

                // Reduce the detection range so that large sized vehicles can pass each other.
                var boxCastExtents = States[stateIndex].Extents * 0.5f;
                boxCastExtents.y *= 1;
                boxCastExtents.z = 0.1f;
                var endPoint = Waypoints[waypointOffset + waypointIndex];

                var distance = Vector3.Distance(startPoint, endPoint);
                var direction = (endPoint - startPoint).normalized;
                var rotation = Quaternion.LookRotation(direction);
                Commands[index] = new BoxcastCommand(
                    startPoint,
                    boxCastExtents,
                    rotation,
                    direction,
                    distance,
                    VehicleLayerMask
                );
            }
        }

        /// <summary>
        /// Calculate distance to the obstacle detected by boxcasts.
        /// </summary>
        private struct CalculateObstacleDistanceJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<RaycastHit> HitInfoArray;
            [ReadOnly] public NativeArray<BoxcastCommand> Commands;
            [ReadOnly] public NativeArray<NativeState> NativeStates;

            public NativeArray<float> Distances;

            public void Execute(int index)
            {
                var hasHit = false;
                var totalDistance = 0f;
                var boxcastCount = Mathf.Min(MaxBoxcastCount, NativeStates[index].WaypointCount);
                for (var commandIndex = index * MaxBoxcastCount;
                     commandIndex < index * MaxBoxcastCount + boxcastCount;
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
        private struct CurveCheckJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<NativeState> NativeStates;
            [ReadOnly] public NativeArray<Vector3> Waypoints;

            [WriteOnly] public NativeArray<bool> IsTurnings;

            public void Execute(int index)
            {
                var state = NativeStates[index];
                var currentForward = Quaternion.AngleAxis(state.Yaw, Vector3.up) * Vector3.forward;
                var currentWaypointIndex = index * MaxWaypointCount;
                var elapsedDistance = Vector3.Distance(state.FrontCenterPosition, Waypoints[currentWaypointIndex]);
                var turnAngle = 0f;
                while (elapsedDistance < 40f)
                {
                    var currentWaypoint = Waypoints[currentWaypointIndex];
                    currentWaypointIndex++;

                    if (currentWaypointIndex >= index * MaxWaypointCount + state.WaypointCount)
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
        private struct RightOfWayCheckJob
        {
            public static float minimumDistanceToIntersection = 18f;

            public static float maximumOverrunStopPointForLaneRules = 1f;

            public static float differenceOrientationDegreesImplyingPerpendicularRoad = 35f;

            // In
            public Transform EGOTransform;

            // In/Out
            public IReadOnlyList<NPCVehicleInternalState> States;

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
                        case NPCVehicleYieldPhase.NONE:
                            refState.YieldLane = null;
                            if (isEnteringIntersection(refState))
                            {
                                refState.YieldPoint = refState.FirstLaneWithIntersection.GetStopPoint();
                                refState.YieldPhase = NPCVehicleYieldPhase.ENTERING_INTERSECTION;
                            }
                            else if (refState.isOnIntersection)
                            {
                                refState.YieldPhase = NPCVehicleYieldPhase.AT_INTERSECTION;
                            }

                            if (refState.YieldPhase != NPCVehicleYieldPhase.NONE)
                                if (refState.isIntersectionWithYieldingLane)
                                    refState.YieldLane = refState.FirstLaneWithIntersection;
                                else
                                    refState.YieldLane = null;
                            break;

                        case NPCVehicleYieldPhase.ENTERING_INTERSECTION:
                            if (refState.isOnIntersection)
                            {
                                refState.YieldPhase = NPCVehicleYieldPhase.AT_INTERSECTION;
                            }
                            else if (ShouldYieldDueToLanes(refState, States, false))
                            {
                                refState.YieldPhase = NPCVehicleYieldPhase.LANES_RULES_ENTERING_INTERSECTION;
                            }
                            else if (isIntersectionBusy(refState, States))
                            {
                                refState.YieldPhase = NPCVehicleYieldPhase.INTERSECTION_BLOCKED;
                            }
                            else if (isLeftHandRuleEnteringIntersection(refState, States))
                            {
                                refState.YieldPhase = NPCVehicleYieldPhase.LEFT_HAND_RULE_ENTERING_INTERSECTION;
                            }
                            break;

                        case NPCVehicleYieldPhase.AT_INTERSECTION:
                            if (!refState.isOnIntersection)
                            {
                                refState.YieldPhase = NPCVehicleYieldPhase.NONE;
                            }
                            else if (ShouldYieldDueToLanes(refState, States, true))
                            {
                                refState.YieldPhase = NPCVehicleYieldPhase.LANES_RULES_AT_INTERSECTION;
                            }
                            else if (isSomeVehicleForcingPriority(refState, States))
                            {
                                refState.YieldPhase = NPCVehicleYieldPhase.FORCING_PRIORITY;
                            }
                            else if (isLeftHandRuleOnIntersection(refState, States))
                            {
                                refState.YieldPhase = NPCVehicleYieldPhase.LEFT_HAND_RULE_AT_INTERSECTION;
                            }
                            break;

                        case NPCVehicleYieldPhase.INTERSECTION_BLOCKED:
                            if (!isIntersectionBusy(refState, States))
                            {
                                refState.YieldPhase = NPCVehicleYieldPhase.NONE;
                            }
                            break;
                        case NPCVehicleYieldPhase.LANES_RULES_ENTERING_INTERSECTION:
                            if (refState.isOnIntersection || !ShouldYieldDueToLanes(refState, States, false))
                            {
                                refState.YieldPhase = NPCVehicleYieldPhase.NONE;
                            }
                            break;
                        case NPCVehicleYieldPhase.LANES_RULES_AT_INTERSECTION:
                            if (!ShouldYieldDueToLanes(refState, States, true))
                            {
                                refState.YieldPhase = NPCVehicleYieldPhase.NONE;
                            }
                            break;
                        case NPCVehicleYieldPhase.LEFT_HAND_RULE_ENTERING_INTERSECTION:
                            if (refState.isOnIntersection || !isLeftHandRuleEnteringIntersection(refState, States))
                            {
                                refState.YieldPhase = NPCVehicleYieldPhase.NONE;
                            }
                            break;
                        case NPCVehicleYieldPhase.LEFT_HAND_RULE_AT_INTERSECTION:
                            if (!isLeftHandRuleOnIntersection(refState, States))
                            {
                                refState.YieldPhase = NPCVehicleYieldPhase.NONE;
                            }
                            break;
                        case NPCVehicleYieldPhase.FORCING_PRIORITY:
                            if (!isSomeVehicleForcingPriority(refState, States))
                            {
                                refState.YieldPhase = NPCVehicleYieldPhase.NONE;
                            }
                            break;
                        default:
                            throw new Exception($"Unattended NPCVehicleYieldPhase case: {refState.YieldPhase}");
                            break;
                    }
                }
            }

            static private bool isYieldingDueToRules(NPCVehicleInternalState refState)
            {
                return refState.YieldPhase == NPCVehicleYieldPhase.LEFT_HAND_RULE_ENTERING_INTERSECTION ||
                refState.YieldPhase == NPCVehicleYieldPhase.LEFT_HAND_RULE_AT_INTERSECTION ||
                refState.YieldPhase == NPCVehicleYieldPhase.LANES_RULES_ENTERING_INTERSECTION ||
                refState.YieldPhase == NPCVehicleYieldPhase.LANES_RULES_AT_INTERSECTION;
            }

            static private bool isYieldingDueToLanesBeforeIntersection(NPCVehicleInternalState refState)
            {
                return
                refState.YieldPhase == NPCVehicleYieldPhase.LEFT_HAND_RULE_ENTERING_INTERSECTION ||
                refState.YieldPhase == NPCVehicleYieldPhase.LANES_RULES_ENTERING_INTERSECTION;
            }

            static private bool theSameIntersectionLane(NPCVehicleInternalState refState, NPCVehicleInternalState otherState)
            {
                return refState.FirstLaneWithIntersection?.name == otherState.FirstLaneWithIntersection?.name;
            }

            static private bool isEnteringFromTheSameSide(NPCVehicleInternalState refState, NPCVehicleInternalState otherState)
            {
                if (otherState.FirstLaneWithIntersection == null || refState.FirstLaneWithIntersection == null)
                    return false;
                else
                    return Vector3.Distance(refState.FirstIntersectionWaypoint.Value, otherState.FirstIntersectionWaypoint.Value) < 3f;
            }

            static private bool isVehicleOnTheLeft(NPCVehicleInternalState refState, NPCVehicleInternalState otherState)
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
            static private bool shouldHavePriority(NPCVehicleInternalState refState, NPCVehicleInternalState otherState)
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
            static private bool shouldBeConsideredForYielding(NPCVehicleInternalState refState, NPCVehicleInternalState otherState)
            {
                return refState.Vehicle.VehicleID != otherState.Vehicle.VehicleID && isEnteringIntersection(otherState);
            }

            /// <summary>
            /// Check if the refState should be consider when examining yielding priority
            /// </summary>
            static private bool isEnteringIntersection(NPCVehicleInternalState refState)
            {
                return refState.DistanceToIntersection <= minimumDistanceToIntersection && !refState.ObstructedByVehicleBehindIntersection;
            }

            static private bool isLeftHandRuleEnteringIntersection(NPCVehicleInternalState refState, IReadOnlyList<NPCVehicleInternalState> states)
            {
                foreach (var otherState in states)
                {
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
                        refState.DominatingVehicle = otherState.Vehicle.RigidBodyTransform;
                        refState.YieldPoint = refState.FollowingLanes[0].GetStopPoint();
                        return true;
                    }
                }
                return false;
            }

            static private bool isLeftHandRuleOnIntersection(NPCVehicleInternalState refState, IReadOnlyList<NPCVehicleInternalState> states)
            {
                foreach (var otherState in states)
                {
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
                        refState.DominatingVehicle = otherState.Vehicle.RigidBodyTransform;
                        refState.YieldPoint = refState.FrontCenterPosition;
                        return true;
                    }
                }
                return false;
            }

            static private bool isIntersectionBusy(NPCVehicleInternalState refState, IReadOnlyList<NPCVehicleInternalState> states)
            {
                foreach (var otherState in states)
                {
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
                        refState.DominatingVehicle = otherState.Vehicle.RigidBodyTransform;
                        refState.YieldPoint = refState.FollowingLanes[0].GetStopPoint();
                        return true;
                    }
                }
                return false;

                static bool isOnPerpendicularRoad(NPCVehicleInternalState refState, NPCVehicleInternalState otherState)
                {
                    var diffAngleDegrees = Mathf.Abs(refState.Yaw - otherState.Yaw);
                    return (diffAngleDegrees < differenceOrientationDegreesImplyingPerpendicularRoad ||
                        diffAngleDegrees > 180f - differenceOrientationDegreesImplyingPerpendicularRoad);
                }
            }

            static private bool isSomeVehicleForcingPriority(NPCVehicleInternalState refState, IReadOnlyList<NPCVehicleInternalState> states)
            {
                foreach (var otherState in states)
                {
                    if (!shouldBeConsideredForYielding(refState, otherState))
                        continue;

                    if (isYieldingDueToRules(otherState))
                        continue;

                    if (!shouldHavePriority(refState, otherState))
                        continue;

                    if (intersectNowFront(refState, otherState))
                    {
                        refState.YieldPoint = refState.FrontCenterPosition;
                        refState.DominatingVehicle = otherState.Vehicle.RigidBodyTransform;
                        return true;
                    }
                }
                return false;
            }


            private bool ShouldYieldDueToLanes(NPCVehicleInternalState refState, IReadOnlyList<NPCVehicleInternalState> states, bool refOnIntersection)
            {
                if (refState.YieldLane == null)
                    return false;

                var isNowYielding = isYieldingDueToRules(refState);
                var stopPoint = refOnIntersection ? (refState.YieldLane.GetStopPoint(1)) : (refState.YieldLane.GetStopPoint());
                // provide hysteresis equal to maximumOverrunStopPointForLaneRules/2.0
                if (isNowYielding && refState.SignedDistanceToPointOnLane(refState.YieldPoint) < -maximumOverrunStopPointForLaneRules)
                    return false;
                else if (!isNowYielding && refState.SignedDistanceToPointOnLane(stopPoint) < -maximumOverrunStopPointForLaneRules / 2.0)
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
                    else if (IsLaneDominatedByVehicle(lane, EGOTransform.position, EGOTransform.forward))
                    {
                        refState.YieldPoint = stopPoint;
                        refState.DominatingVehicle = EGOTransform;
                        return true;
                    }
                }
                return false;


                static bool IsLaneDominatedByAny(TrafficLane lane, IReadOnlyList<NPCVehicleInternalState> states,
                    NPCVehicleInternalState refState, bool refOnIntersection, out Transform dominatingVehicle)
                {
                    dominatingVehicle = null;
                    foreach (var otherState in states)
                    {
                        if (!shouldBeConsideredForYielding(refState, otherState))
                            continue;

                        if (otherState.yieldingPriorityAtTrafficLight)
                            continue;

                        if (refOnIntersection && (isYieldingDueToRules(otherState) || otherState.YieldPhase == NPCVehicleYieldPhase.INTERSECTION_BLOCKED))
                            continue;

                        if (!IsLaneDominatedBy(lane, otherState))
                            continue;

                        if (!intersectOverall(refState, otherState))
                            continue;

                        dominatingVehicle = otherState.Vehicle.RigidBodyTransform;
                        return true;
                    }
                    return false;
                }

                static bool IsLaneDominatedBy(TrafficLane lane, NPCVehicleInternalState state)
                {
                    return (state.CurrentFollowingLane.intersectionLane || state.IsNextLaneIntersection) && state.FirstLaneWithIntersection == lane;
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

            static private bool intersectOverall(NPCVehicleInternalState refState, NPCVehicleInternalState otherState)
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

            static private bool intersectNowFront(NPCVehicleInternalState refState, NPCVehicleInternalState otherState)
            {
                if (!refState.CurrentFollowingLane.intersectionLane || !otherState.CurrentFollowingLane.intersectionLane)
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
        private struct TrafficLightCheckJob
        {
            // In/Out
            public IReadOnlyList<NPCVehicleInternalState> States;

            public void Execute()
            {
                foreach (var state in States)
                {
                    if (state.ShouldDespawn)
                        continue;

                    // Find next stop line with a traffic light.
                    // If the vehicle is stopping, use cached stop line.
                    // This is in case a vehicle crosses the stop line while stopping.
                    if (state.TrafficLightPassability != TrafficLightPassability.RED)
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
                        state.TrafficLightPassability = TrafficLightPassability.GREEN;
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
        /// Write native data to <see cref="NPCVehicleInternalState"/>.
        /// This job is sequential.
        /// </summary>
        private struct WriteStateJob
        {
            [ReadOnly] public NativeArray<RaycastHit> GroundHitInfoArray;
            [ReadOnly] public NativeArray<float> ObstacleDistances;
            [ReadOnly] public NativeArray<bool> IsTurnings;

            public IReadOnlyList<NPCVehicleInternalState> States;

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
        private const int MaxWaypointCount = 10;

        /// <summary>
        /// Maximum number of boxcasts executed per vehicle.
        /// Calculation cost is proportional to this value.
        /// This value should be less than or equal to <see cref="MaxWaypointCount"/>.
        /// </summary>
        private const int MaxBoxcastCount = 5;

        private readonly LayerMask vehicleLayerMask;
        private readonly LayerMask groundLayerMask;

        // Native states
        private NativeArray<NativeState> nativeStates;
        private NativeArray<Vector3> waypoints;

        // Ground Check
        private NativeArray<RaycastCommand> raycastCommands;
        private NativeArray<RaycastHit> groundHitInfoArray;

        // Obstacle Check
        private NativeArray<BoxcastCommand> boxcastCommands;
        private NativeArray<RaycastHit> obstacleHitInfoArray;
        private NativeArray<float> obstacleDistances;

        // Curve Check
        private NativeArray<bool> isTurnings;

        // Job Handles
        private JobHandle groundCheckJobHandle;
        private JobHandle raycastJobHandle;
        private JobHandle obstacleCheckJobHandle;
        private JobHandle boxcastJobHandle;
        private JobHandle calculateObstacleDistanceJobHandle;
        private JobHandle curveCheckJobHandle;

        public NPCVehicleCognitionStep(LayerMask vehicleLayerMask, LayerMask groundLayerMask, int maxVehicleCount)
        {
            this.vehicleLayerMask = vehicleLayerMask;
            this.groundLayerMask = groundLayerMask;

            nativeStates = new NativeArray<NativeState>(maxVehicleCount, Allocator.Persistent);
            waypoints = new NativeArray<Vector3>(maxVehicleCount * MaxWaypointCount, Allocator.Persistent);

            raycastCommands = new NativeArray<RaycastCommand>(maxVehicleCount, Allocator.Persistent);
            groundHitInfoArray = new NativeArray<RaycastHit>(maxVehicleCount, Allocator.Persistent);

            boxcastCommands = new NativeArray<BoxcastCommand>(maxVehicleCount * MaxBoxcastCount, Allocator.Persistent);
            obstacleHitInfoArray =
                new NativeArray<RaycastHit>(maxVehicleCount * MaxBoxcastCount, Allocator.Persistent);
            obstacleDistances = new NativeArray<float>(maxVehicleCount, Allocator.Persistent);

            isTurnings = new NativeArray<bool>(maxVehicleCount, Allocator.Persistent);
        }

        public void Dispose()
        {
            var dependsOn = JobHandle.CombineDependencies(
                raycastJobHandle, calculateObstacleDistanceJobHandle, curveCheckJobHandle);

            nativeStates.Dispose(dependsOn);
            waypoints.Dispose(dependsOn);

            raycastCommands.Dispose(dependsOn);
            groundHitInfoArray.Dispose(dependsOn);

            boxcastCommands.Dispose(dependsOn);
            obstacleHitInfoArray.Dispose(dependsOn);
            obstacleDistances.Dispose(dependsOn);

            isTurnings.Dispose(dependsOn);
        }

        public void Execute(
            IReadOnlyList<NPCVehicleInternalState> states,
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
                Results = nativeStates,
                Waypoints = waypoints,
                States = states
            }.Execute();

            Profiler.EndSample();

            Profiler.BeginSample("Cognition.JobSchedule");

            obstacleCheckJobHandle =
                new ObstacleCheckJob
                {
                    Commands = boxcastCommands,
                    States = nativeStates,
                    VehicleLayerMask = vehicleLayerMask,
                    Waypoints = waypoints
                }.Schedule(boxcastCommands.Length, 16);

            boxcastJobHandle =
                BoxcastCommand.ScheduleBatch(
                    boxcastCommands,
                    obstacleHitInfoArray,
                    16,
                    obstacleCheckJobHandle);

            // Start background jobs
            JobHandle.ScheduleBatchedJobs();

            groundCheckJobHandle =
                new GroundCheckJob
                {
                    Commands = raycastCommands,
                    GroundLayerMask = groundLayerMask,
                    NativeStates = nativeStates
                }.Schedule(nativeStates.Length, 8);

            raycastJobHandle =
                RaycastCommand.ScheduleBatch(
                    raycastCommands,
                    groundHitInfoArray,
                    8,
                    groundCheckJobHandle);

            calculateObstacleDistanceJobHandle = new CalculateObstacleDistanceJob
            {
                HitInfoArray = obstacleHitInfoArray,
                Commands = boxcastCommands,
                NativeStates = nativeStates,
                Distances = obstacleDistances
            }.Schedule(nativeStates.Length, 8, boxcastJobHandle);

            curveCheckJobHandle = new CurveCheckJob
            {
                NativeStates = nativeStates,
                Waypoints = waypoints,
                IsTurnings = isTurnings
            }.Schedule(nativeStates.Length, 8);

            // Start rest background jobs
            JobHandle.ScheduleBatchedJobs();

            Profiler.EndSample();
            Profiler.BeginSample("Cognition.CheckRightOfWay");

            new RightOfWayCheckJob
            {
                EGOTransform = egoTransform,
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

            obstacleCheckJobHandle.Complete();
            groundCheckJobHandle.Complete();
            boxcastJobHandle.Complete();
            raycastJobHandle.Complete();
            calculateObstacleDistanceJobHandle.Complete();
            curveCheckJobHandle.Complete();

            Profiler.EndSample();
            Profiler.BeginSample("Cognition.WriteState");

            new WriteStateJob
            {
                GroundHitInfoArray = groundHitInfoArray,
                ObstacleDistances = obstacleDistances,
                IsTurnings = isTurnings,
                States = states
            }.Execute();

            // if vehicle is far from the ego vehicle, it should be despawned
            foreach (var state in states)
            {
                if ((state.Vehicle.transform.position - egoTransform.position).magnitude > 600f)
                    state.ShouldDespawn = true;
            }

            Profiler.EndSample();
        }

        public void ShowGizmos(IReadOnlyList<NPCVehicleInternalState> states, bool showYieldingPhase, bool showObstacleChecking)
        {
            if (showYieldingPhase)
            {
                foreach (var state in states)
                {
                    var stateCurrentPosition = state.FrontCenterPosition;
                    stateCurrentPosition.y = state.Vehicle.transform.position.y + 2f;

                    if (state.YieldPhase == NPCVehicleYieldPhase.NONE ||
                        state.YieldPhase == NPCVehicleYieldPhase.ENTERING_INTERSECTION ||
                        state.YieldPhase == NPCVehicleYieldPhase.AT_INTERSECTION)
                    {
                        continue;
                    }

                    switch (state.YieldPhase)
                    {
                        case NPCVehicleYieldPhase.INTERSECTION_BLOCKED:
                            Gizmos.color = Color.blue;
                            break;
                        case NPCVehicleYieldPhase.LEFT_HAND_RULE_ENTERING_INTERSECTION:
                            Gizmos.color = Color.gray;
                            break;
                        case NPCVehicleYieldPhase.LEFT_HAND_RULE_AT_INTERSECTION:
                            Gizmos.color = Color.black;
                            break;
                        case NPCVehicleYieldPhase.LANES_RULES_ENTERING_INTERSECTION:
                            Gizmos.color = Color.yellow;
                            break;
                        case NPCVehicleYieldPhase.LANES_RULES_AT_INTERSECTION:
                            Gizmos.color = Color.red;
                            break;
                        case NPCVehicleYieldPhase.FORCING_PRIORITY:
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

                    var boxcastCount = Mathf.Min(MaxBoxcastCount, nativeStates[stateIndex].WaypointCount);
                    for (var commandIndex = stateIndex * MaxBoxcastCount;
                            commandIndex < stateIndex * MaxBoxcastCount + boxcastCount;
                            commandIndex++)
                    {
                        var hitInfo = obstacleHitInfoArray[commandIndex];
                        var hasHit = hitInfo.collider != null;

                        var command = boxcastCommands[commandIndex];
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
