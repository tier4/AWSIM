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
                boxCastExtents.y *= 3;
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
                foreach (var state in States)
                {
                    if (state.ShouldDespawn)
                        continue;

                    switch (state.YieldPhase)
                    {
                        case NPCVehicleYieldPhase.NONE:
                            // Check if the next lane is a yielding lane.
                            if (state.FollowingLanes.Count > 1)
                            {
                                var nextLane = state.FollowingLanes[1];
                                var isEnteringYieldingLane = nextLane.RightOfWayLanes.Count > 0;
                                if (isEnteringYieldingLane)
                                {
                                    state.YieldPhase = NPCVehicleYieldPhase.ENTERING_YIELDING_LANE;
                                    state.YieldPoint = GetStopPoint(nextLane);
                                    state.YieldLane = nextLane;
                                    break;
                                }
                            }
                            // Check if the current lane is a yielding lane.
                            if (state.FollowingLanes.Count > 0)
                            {
                                var currentLine = state.FollowingLanes[0];
                                if (currentLine.RightOfWayLanes.Count > 0)
                                {
                                    state.YieldPhase = NPCVehicleYieldPhase.ON_YIELDING_LANE;
                                    state.YieldPoint = GetStopPoint(currentLine, 1);
                                    state.YieldLane = currentLine;
                                    break;
                                }
                            }
                            break;
                        case NPCVehicleYieldPhase.ON_YIELDING_LANE:
                        case NPCVehicleYieldPhase.ENTERING_YIELDING_LANE:
                            // Do nothing if the vehicle is far from stop line
                            var signedDistanceToStopLine = state.SignedDistanceToPointOnLane(state.YieldPoint);
                            if (signedDistanceToStopLine >= 3f)
                                break;
                            var shouldYield = ShouldYield(state, States, EGOTransform, out var dominatingVehicle);
                            state.DominatingVehicle = dominatingVehicle;
                            if (shouldYield)
                                state.YieldPhase = NPCVehicleYieldPhase.YIELDING;

                            // Cancel yielding if the vehicle exceeds stop line
                            if (signedDistanceToStopLine < -2f)
                                state.YieldPhase = NPCVehicleYieldPhase.NONE;
                            break;

                        case NPCVehicleYieldPhase.YIELDING:
                            shouldYield = ShouldYield(state, States, EGOTransform, out dominatingVehicle);
                            state.DominatingVehicle = dominatingVehicle;
                            if (!shouldYield)
                                state.YieldPhase = NPCVehicleYieldPhase.NONE;
                            break;
                    }
                }
            }

            private static Vector3 GetStopPoint(TrafficLane lane, int waypointIndex = 0)
            {
                return lane.StopLine == null
                    ? lane.Waypoints[waypointIndex]
                    : lane.StopLine.CenterPoint;
            }

            private static bool ShouldYield(
                NPCVehicleInternalState state, IReadOnlyList<NPCVehicleInternalState> states,
                Transform egoTransform, out Transform dominatingVehicle)
            {
                dominatingVehicle = null;
                foreach (var lane in state.YieldLane.RightOfWayLanes)
                {
                    var isDominatedByNPC = IsLaneDominatedByAny(lane, states, state, out dominatingVehicle);
                    if (isDominatedByNPC)
                        return true;

                    var isDominatedByEGO =
                        egoTransform != null &&
                        IsLaneDominatedBy(lane, egoTransform.position, egoTransform.forward);
                    if (isDominatedByEGO)
                    {
                        dominatingVehicle = egoTransform;
                        return true;
                    }
                }

                return false;
            }

            private static bool IsLaneDominatedByAny(TrafficLane lane, IReadOnlyList<NPCVehicleInternalState> states,
                NPCVehicleInternalState refState, out Transform dominatingVehicle)
            {
                foreach (var state in states)
                {
                    if (!IsLaneDominatedBy(lane, state))
                        continue;
                    if (haveTheVehiclesPassed(refState, state))
                        continue;
                    dominatingVehicle = state.Vehicle.transform;
                    return true;
                }

                dominatingVehicle = null;
                return false;

                static bool haveTheVehiclesPassed(NPCVehicleInternalState refState, NPCVehicleInternalState state)
                {
                    var refPosition = refState.BackCenterPosition;
                    var position = state.BackCenterPosition;
                    refPosition.y = 0f;
                    position.y = 0f;
                    return Vector3.Dot(refState.Forward, position - refPosition) < 0f;
                }
            }

            /// <summary>
            /// Check if <paramref name="lane"/> is dominated by <paramref name="state"/>.
            /// All vehicles trying to pass on a non-priority lane needs to yield if any vehicle dominates the right of way lane.
            /// </summary>
            private static bool IsLaneDominatedBy(TrafficLane lane, NPCVehicleInternalState state)
            {
                if (state.CurrentFollowingLane != lane)
                    return false;

                // Ignore vehicles that will not enter the lane soon.
                var isWaitingTrafficLight = state.TrafficLightPassability == TrafficLightPassability.RED;
                return !isWaitingTrafficLight;
            }

            /// <summary>
            /// Check if <paramref name="lane"/> is dominated by a vehicle whose position is <paramref name="vehiclePosition"/> and forward direction is <paramref name="vehicleForward"/>.
            /// All vehicles trying to pass on a non-priority lane needs to yield if any vehicle dominates the right of way lane.<br/>
            /// It is implemented for vehicles for which lane information is not explicitly maintained, such as EGO vehicles.
            /// </summary>
            private static bool IsLaneDominatedBy(TrafficLane lane, Vector3 vehiclePosition, Vector3 vehicleForward)
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

            Profiler.EndSample();
        }

        public void ShowGizmos(IReadOnlyList<NPCVehicleInternalState> states)
        {
            foreach (var state in states)
            {
                var currentPosition = state.FrontCenterPosition;
                currentPosition.y = state.Vehicle.transform.position.y + 1f;

                // Yield
                currentPosition.y += 3f;
                switch (state.YieldPhase)
                {
                    case NPCVehicleYieldPhase.NONE:
                        break;
                    case NPCVehicleYieldPhase.ENTERING_YIELDING_LANE:
                        Gizmos.color = Color.yellow;

                        Gizmos.DrawSphere(currentPosition, 0.4f);
                        break;
                    case NPCVehicleYieldPhase.YIELDING:
                        Gizmos.color = Color.red;
                        Gizmos.DrawSphere(currentPosition, 0.4f);
                        if (state.DominatingVehicle != null)
                            Gizmos.DrawLine(currentPosition, state.DominatingVehicle.transform.position);
                        break;
                }
            }

            // Obstacle checking
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
