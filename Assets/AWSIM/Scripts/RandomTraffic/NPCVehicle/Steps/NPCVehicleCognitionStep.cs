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

		public static int refID = -151;
		public static int stateID = -23;
		// if (refState.Vehicle.VehicleID == 19 && state.Vehicle.VehicleID == 4)
		// 	Debug.Log($"{refState.Vehicle.VehicleID}x{state.Vehicle.VehicleID} isLeftHandRuleOnIntersection here 1");


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
							lane.Waypoints.Length - srcIndex
						);

						NativeArray<Vector3>.Copy(
							lane.Waypoints,
							srcIndex,
							Waypoints,
							offset + dstIndex,
							length
						);

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
			[ReadOnly]
			public NativeArray<NativeState> NativeStates;

			[WriteOnly]
			public NativeArray<RaycastCommand> Commands;

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

			[ReadOnly]
			public NativeArray<NativeState> States;

			[ReadOnly]
			public NativeArray<Vector3> Waypoints;

			[WriteOnly]
			public NativeArray<BoxcastCommand> Commands;

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

				var startPoint =
					waypointIndex == 0
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
			[ReadOnly]
			public NativeArray<RaycastHit> HitInfoArray;

			[ReadOnly]
			public NativeArray<BoxcastCommand> Commands;

			[ReadOnly]
			public NativeArray<NativeState> NativeStates;

			public NativeArray<float> Distances;

			public void Execute(int index)
			{
				var hasHit = false;
				var totalDistance = 0f;
				var boxcastCount = Mathf.Min(MaxBoxcastCount, NativeStates[index].WaypointCount);
				for (
					var commandIndex = index * MaxBoxcastCount;
					commandIndex < index * MaxBoxcastCount + boxcastCount;
					commandIndex++
				)
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

				Distances[index] = hasHit ? totalDistance : float.MaxValue;
			}
		}

		/// <summary>
		/// Check sharp curve existence.
		/// </summary>
		private struct CurveCheckJob : IJobParallelFor
		{
			[ReadOnly]
			public NativeArray<NativeState> NativeStates;

			[ReadOnly]
			public NativeArray<Vector3> Waypoints;

			[WriteOnly]
			public NativeArray<bool> IsTurnings;

			public void Execute(int index)
			{
				var state = NativeStates[index];
				var currentForward = Quaternion.AngleAxis(state.Yaw, Vector3.up) * Vector3.forward;
				var currentWaypointIndex = index * MaxWaypointCount;
				var elapsedDistance = Vector3.Distance(
					state.FrontCenterPosition,
					Waypoints[currentWaypointIndex]
				);
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

			public static float maximumOverrunStopPointForLaneRules = 3f;

			// In
			public Transform EGOTransform;

			// In/Out
			public IReadOnlyList<NPCVehicleInternalState> States;

			static bool needToYield(NPCVehicleInternalState refState, NPCVehicleInternalState otherState)
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

			private bool isIntersectionBusy(NPCVehicleInternalState refState, IReadOnlyList<NPCVehicleInternalState> states)
			{
				foreach (var state in states)
				{
					if (refState.Vehicle.VehicleID == state.Vehicle.VehicleID)
						continue;
					if (state.BehindVehicleBeforeIntersection)
						continue;
					if (state.DistanceToIntersection > minimumDistanceToIntersection)
						continue;

					if (!state.isOnIntersection)
						continue;

					if (refState.Vehicle.VehicleID == refID && state.Vehicle.VehicleID == stateID)
					{
						Debug.Log($"{refState.Vehicle.VehicleID}x{state.Vehicle.VehicleID} aft isOnIntersection ");
					}

					if (isYieldingDueToLanes(state))
						continue;

					if (refState.Vehicle.VehicleID == refID && state.Vehicle.VehicleID == stateID)
					{
						Debug.Log($"{refState.Vehicle.VehicleID}x{state.Vehicle.VehicleID} aft isYieldingDueToLanes ");
					}

					if (!refState.intersectOverall(state))
						continue;

					if (refState.Vehicle.VehicleID == refID && state.Vehicle.VehicleID == stateID)
					{
						Debug.Log($"{refState.Vehicle.VehicleID}x{state.Vehicle.VehicleID} aft intersectOverall ");
					}


					if (!theSameIntersectionLane(refState, state) && !turnInTheSameWay(refState, state))
					{
						refState.DominatingVehicle = state.Vehicle.RigidBodyTransform;
						refState.YieldPoint = refState.FollowingLanes[0].GetStopPoint();
						return true;
					}
				}
				return false;
			}

			private bool isLeftHandRuleEnteringIntersection(
				NPCVehicleInternalState refState,
				IReadOnlyList<NPCVehicleInternalState> states
			)
			{
				foreach (var state in states)
				{
					if (refState.Vehicle.VehicleID == state.Vehicle.VehicleID)
						continue;
					if (state.BehindVehicleBeforeIntersection)
						continue;
					if (state.DistanceToIntersection > minimumDistanceToIntersection)
						continue;

					if (!state.isOnIntersection && !state.isEnteringIntersection)
						continue;

					if (isYieldingBeforeIntersection(state))
						continue;

					if (state.YieldPhase == NPCVehicleYieldPhase.LANES_RULES_AT_INTERSECTION)
						continue;


					if (!refState.intersectOverall(state))
						continue;

					if (isVehicleOnTheLeft(refState, state) && !isVehicleOnBehindAndFollow(refState, state)
						&& !state.yieldingPriorityAtTrafficLight && !turnInTheSameWay(refState, state))
					{
						refState.DominatingVehicle = state.Vehicle.RigidBodyTransform;
						refState.YieldPoint = refState.FollowingLanes[0].GetStopPoint();
						return true;
					}
				}
				return false;
			}

			private bool isYieldingDueToLanes(NPCVehicleInternalState refState)
			{
				return refState.YieldPhase == NPCVehicleYieldPhase.LEFT_HAND_RULE_ENTERING_INTERSECTION ||
				refState.YieldPhase == NPCVehicleYieldPhase.LEFT_HAND_RULE_AT_INTERSECTION ||
				refState.YieldPhase == NPCVehicleYieldPhase.LANES_RULES_ENTERING_INTERSECTION ||
				refState.YieldPhase == NPCVehicleYieldPhase.LANES_RULES_AT_INTERSECTION;
				// refState.YieldPhase == NPCVehicleYieldPhase.INTERSECTION_BLOCKED ||
				// refState.YieldPhase == NPCVehicleYieldPhase.FORCING_PRIORITY;
			}

			private bool isYieldingBeforeIntersection(NPCVehicleInternalState refState)
			{
				return
				refState.YieldPhase == NPCVehicleYieldPhase.LEFT_HAND_RULE_ENTERING_INTERSECTION ||
				refState.YieldPhase == NPCVehicleYieldPhase.LANES_RULES_ENTERING_INTERSECTION;
			}

			private bool isLeftHandRuleOnIntersection(
				NPCVehicleInternalState refState,
				IReadOnlyList<NPCVehicleInternalState> states
			)
			{
				foreach (var state in states)
				{
					if (refState.Vehicle.VehicleID == state.Vehicle.VehicleID)
						continue;
					if (state.BehindVehicleBeforeIntersection)
						continue;
					if (state.DistanceToIntersection > minimumDistanceToIntersection)
						continue;


					if (!state.isOnIntersection)
						continue;

					if (isYieldingDueToLanes(state))
						continue;

					if (!refState.intersectNowFront(state))
						continue;

					if (isVehicleOnTheLeft(refState, state) && !isVehicleOnBehindAndFollow(refState, state)
						&& !turnInTheSameWay(refState, state) && !theSameIntersectionLane(refState, state)
						&& !needToYield(state, refState))
					{
						refState.DominatingVehicle = state.Vehicle.RigidBodyTransform;
						refState.YieldPoint = refState.FrontCenterPosition;
						return true;
					}
				}
				return false;
			}

			public bool isSomeVehicleForcingPriority(
				NPCVehicleInternalState refState,
				IReadOnlyList<NPCVehicleInternalState> states
			)
			{
				foreach (var someState in states)
				{
					if (refState.Vehicle.VehicleID == someState.Vehicle.VehicleID)
						continue;
					if (refState.BehindVehicleBeforeIntersection)
						break;
					if (someState.DistanceToIntersection > minimumDistanceToIntersection)
						continue;

					if (someState.FollowingLanes.Count == 0)
						continue;
					if (someState.CurrentFollowingLane.RightOfWayLanes.Count == 0)
						continue;
					if (isYieldingDueToLanes(someState))
						continue;

					if (needToYield(refState, someState) && refState.intersectNowFront(someState))
					{
						refState.YieldPoint = refState.FrontCenterPosition;
						refState.YieldLane = someState.CurrentFollowingLane;
						refState.DominatingVehicle = someState.Vehicle.RigidBodyTransform;
						return true;
					}
				}
				return false;
			}

			bool theSameIntersectionLane(
				NPCVehicleInternalState refState,
				NPCVehicleInternalState state
			)
			{
				if (state.FirstLaneWithIntersection == null || refState.FirstLaneWithIntersection == null)
					return false;
				return refState.FirstLaneWithIntersection.name == state.FirstLaneWithIntersection.name;
			}

			bool turnInTheSameWay(NPCVehicleInternalState refState, NPCVehicleInternalState state)
			{
				if (state.FirstLaneWithIntersection == null || refState.FirstLaneWithIntersection == null)
					return false;

				return Vector3.Distance(refState.FirstIntersectionWaypoint.Value, state.FirstIntersectionWaypoint.Value) < 3f;
			}

			bool isVehicleOnTheLeft(NPCVehicleInternalState refState, NPCVehicleInternalState state)
			{
				var refPosition = refState.FrontCenterPosition;
				var positionBack = state.BackCenterPosition;
				var positionFront = state.FrontCenterPosition;
				refPosition.y = 0f;
				positionBack.y = 0f;
				positionFront.y = 0f;
				var crossFront = Vector3.Cross(refState.Forward, positionFront - refPosition).y;
				var crossBack = Vector3.Cross(refState.Forward, positionBack - refPosition).y;
				return crossFront < 0f || crossBack < 0f;
			}

			bool isVehicleOnBehindAndFollow(NPCVehicleInternalState refState, NPCVehicleInternalState state)
			{
				var refPosition = refState.FrontCenterPosition;
				var positionBack = state.BackCenterPosition;
				var positionFront = state.FrontCenterPosition;
				refPosition.y = 0f;
				positionBack.y = 0f;
				positionFront.y = 0f;
				var dotBack = Vector3.Dot(refState.Forward, positionBack - refPosition);
				var dotFront = Vector3.Dot(refState.Forward, positionFront - refPosition);
				return (dotBack < 0f || dotFront < 0f) && Vector3.Angle(refState.Forward, state.Forward) < 90f;
			}

			private bool ShouldYield(
				NPCVehicleInternalState refState,
				IReadOnlyList<NPCVehicleInternalState> states, bool refOnIntersection
			)
			{
				Transform dominatingVehicle = null;
				if (refState.YieldLane == null)
					return false;

				foreach (var lane in refState.YieldLane.RightOfWayLanes)
				{
					if (IsLaneDominatedByAny(lane, states, refState, refOnIntersection, out dominatingVehicle))
					{
						refState.DominatingVehicle = dominatingVehicle;
						return true;
					}
					else if (IsLaneDominatedBy(lane, EGOTransform.position, EGOTransform.forward))
					{
						refState.DominatingVehicle = EGOTransform;
						return true;
					}
				}
				return false;
			}

			bool IsLaneDominatedByAny(
				TrafficLane lane,
				IReadOnlyList<NPCVehicleInternalState> states,
				NPCVehicleInternalState refState, bool refOnIntersection,
				out Transform dominatingVehicle
			)
			{
				dominatingVehicle = null;
				foreach (var state in states)
				{
					if (refState.Vehicle.VehicleID == state.Vehicle.VehicleID)
						continue;
					if (state.BehindVehicleBeforeIntersection)
						continue;
					if (state.DistanceToIntersection > minimumDistanceToIntersection)
						continue;


					if (refState.Vehicle.VehicleID == 35 && (state.Vehicle.VehicleID == stateID || state.Vehicle.VehicleID == 43))
					{
						var refPosition = refState.FrontCenterPosition;
						var positionFront = state.FrontCenterPosition;
						var dotFront = Vector3.Dot(refState.Forward, positionFront - refPosition);
						var angle = Vector3.Angle(refState.Forward, state.Forward);
						Debug.Log($"{refState.Vehicle.VehicleID}x{state.Vehicle.VehicleID}: {dotFront}, {angle}");
					}

					if (state.yieldingPriorityAtTrafficLight)
						continue;
					if (refOnIntersection && isYieldingDueToLanes(state))
						continue;
					if (!IsLaneDominatedBy(lane, state))
						continue;
					if (!refState.intersectOverall(state))
						continue;
					dominatingVehicle = state.Vehicle.RigidBodyTransform;
					return true;
				}
				return false;
			}

			static bool IsLaneDominatedBy(TrafficLane lane, NPCVehicleInternalState state)
			{
				if (state.FirstLaneWithIntersection == lane && IsDistanceToStopPointExceeded(state))
					return true;
				return false;

				static bool IsDistanceToStopPointExceeded(NPCVehicleInternalState refState)
				{
					if (refState.CurrentFollowingLane.intersectionLane)
						return true;
					else if (!refState.IsNextLaneIntersection())
						return false;
					else
						return true;
				}
			}

			/// Check if <paramref name="lane"/> is dominated by a vehicle whose position is <paramref name="vehiclePosition"/> and forward direction is <paramref name="vehicleForward"/>.
			/// All vehicles trying to pass on a non-priority lane needs to yield if any vehicle dominates the right of way lane.<br/>
			/// It is implemented for vehicles for which lane information is not explicitly maintained, such as EGO vehicles.
			/// </summary>
			static bool IsLaneDominatedBy(
				TrafficLane lane,
				Vector3 vehiclePosition,
				Vector3 vehicleForward
			)
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


			/// <summary>
			/// Checks traffic conditions on right of ways.
			/// This method consists of the following steps:<br/>
			/// - Find
			/// </summary>
			public void Execute()
			{
				foreach (var refState in States)
				{
					var here = "here";
					try
					{
						if (refState.ShouldDespawn)
							continue;

						if (refState.Vehicle.VehicleID == refID || refState.Vehicle.VehicleID == stateID)
						{
							Debug.Log($"{refState.Vehicle.VehicleID} YieldPhase: {refState.YieldPhase}");
						}
						switch (refState.YieldPhase)
						{
							case NPCVehicleYieldPhase.NONE:
								if (refState.isEnteringIntersection)
								{
									if (refState.Vehicle.VehicleID == 16)
										Debug.Log($"{refState.Vehicle.VehicleID} distance to front: {refState.DistanceToFrontVehicle}");

									if (refState.BehindVehicleBeforeIntersection)
										break;
									if (refState.DistanceToIntersection > minimumDistanceToIntersection)
										break;

									if (refState.isEnteringYieldingLane)
										refState.YieldLane = refState.FirstLaneWithIntersection;
									else
										refState.YieldLane = null;
									refState.YieldPoint = refState.FirstLaneWithIntersection.GetStopPoint();
									refState.YieldPhase =
										NPCVehicleYieldPhase.ENTERING_INTERSECTION;
									break;
								}
								else if (refState.isOnIntersection)
								{
									if (refState.isEnteringYieldingLane)
										refState.YieldLane = refState.FirstLaneWithIntersection;
									else
										refState.YieldLane = null;
									refState.YieldPhase = NPCVehicleYieldPhase.AT_INTERSECTION;
									break;
								}
								break;
							case NPCVehicleYieldPhase.ENTERING_INTERSECTION:
								if (refState.isOnIntersection)
								{
									refState.YieldPhase = NPCVehicleYieldPhase.AT_INTERSECTION;
									break;
								}
								if (refState.YieldLane != null && ShouldYield(refState, States, false))
								{
									if (refState.SignedDistanceToPointOnLane(refState.YieldLane.GetStopPoint()) < -maximumOverrunStopPointForLaneRules / 2.0)
										break;
									refState.YieldLane = refState.FirstLaneWithIntersection;
									refState.YieldPoint = refState.YieldLane.GetStopPoint();
									refState.YieldPhase =
										NPCVehicleYieldPhase.LANES_RULES_ENTERING_INTERSECTION;
									break;
								}
								if (isIntersectionBusy(refState, States))
								{
									refState.YieldPhase = NPCVehicleYieldPhase.INTERSECTION_BLOCKED;
									break;
								}
								if (isLeftHandRuleEnteringIntersection(refState, States))
								{
									refState.YieldPhase =
										NPCVehicleYieldPhase.LEFT_HAND_RULE_ENTERING_INTERSECTION;
									break;
								}
								break;
							case NPCVehicleYieldPhase.AT_INTERSECTION:
								if (!refState.isOnIntersection)
								{
									refState.YieldPhase = NPCVehicleYieldPhase.NONE;
									break;
								}
								if (refState.YieldLane != null && ShouldYield(refState, States, true))
								{
									if (refState.SignedDistanceToPointOnLane(refState.YieldLane.GetStopPoint(1)) < -maximumOverrunStopPointForLaneRules / 2.0)
										break;
									refState.YieldLane = refState.CurrentFollowingLane;
									refState.YieldPoint = refState.YieldLane.GetStopPoint(1);
									refState.YieldPhase =
										NPCVehicleYieldPhase.LANES_RULES_AT_INTERSECTION;
									break;
								}
								if (isSomeVehicleForcingPriority(refState, States))
								{
									refState.YieldPhase =
										NPCVehicleYieldPhase.FORCING_PRIORITY;
									break;
								}
								if (isLeftHandRuleOnIntersection(refState, States))
								{
									refState.YieldPhase =
										NPCVehicleYieldPhase.LEFT_HAND_RULE_AT_INTERSECTION;
									break;
								}
								break;
							case NPCVehicleYieldPhase.INTERSECTION_BLOCKED:
								if (!isIntersectionBusy(refState, States))
									refState.YieldPhase = NPCVehicleYieldPhase.NONE;
								break;
							case NPCVehicleYieldPhase.LANES_RULES_ENTERING_INTERSECTION:
							case NPCVehicleYieldPhase.LANES_RULES_AT_INTERSECTION:
								if (
									!ShouldYield(refState, States, refState.YieldPhase == NPCVehicleYieldPhase.LANES_RULES_AT_INTERSECTION)
									|| refState.SignedDistanceToPointOnLane(refState.YieldPoint)
										< -maximumOverrunStopPointForLaneRules
								)
								{
									refState.YieldLane = null;
									refState.YieldPhase = NPCVehicleYieldPhase.NONE;
								}
								break;
							case NPCVehicleYieldPhase.LEFT_HAND_RULE_ENTERING_INTERSECTION:
								if (!isLeftHandRuleEnteringIntersection(refState, States))
									refState.YieldPhase = NPCVehicleYieldPhase.NONE;
								break;
							case NPCVehicleYieldPhase.LEFT_HAND_RULE_AT_INTERSECTION:
								if (!isLeftHandRuleOnIntersection(refState, States))
									refState.YieldPhase = NPCVehicleYieldPhase.NONE;
								break;
							case NPCVehicleYieldPhase.FORCING_PRIORITY:
								if (!isSomeVehicleForcingPriority(refState, States))
									refState.YieldPhase = NPCVehicleYieldPhase.NONE;
								break;
							default:
								Debug.Log("### ERROR CASES!!");
								break;
						}
					}
					catch (InvalidOperationException ex)
					{
						Debug.Log(
							$"{refState.Vehicle.VehicleID} ### InvalidOperationException: {here} "
								+ ex.Message
						);
					}
					catch (NullReferenceException ex)
					{
						Debug.Log(
							$"{refState.Vehicle.VehicleID} ### NullReferenceException: {here} "
								+ ex.Message
						);
					}
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
					state.TrafficLightPassability = RandomTrafficUtils.GetPassability(
						trafficLight,
						intersectionLane.TurnDirection
					);
				}
			}
		}

		/// <summary>
		/// Write native data to <see cref="NPCVehicleInternalState"/>.
		/// This job is sequential.
		/// </summary>
		private struct WriteStateJob
		{
			[ReadOnly]
			public NativeArray<RaycastHit> GroundHitInfoArray;

			[ReadOnly]
			public NativeArray<float> ObstacleDistances;

			[ReadOnly]
			public NativeArray<bool> IsTurnings;

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

		public NPCVehicleCognitionStep(
			LayerMask vehicleLayerMask,
			LayerMask groundLayerMask,
			int maxVehicleCount
		)
		{
			this.vehicleLayerMask = vehicleLayerMask;
			this.groundLayerMask = groundLayerMask;

			nativeStates = new NativeArray<NativeState>(maxVehicleCount, Allocator.Persistent);
			waypoints = new NativeArray<Vector3>(
				maxVehicleCount * MaxWaypointCount,
				Allocator.Persistent
			);

			raycastCommands = new NativeArray<RaycastCommand>(
				maxVehicleCount,
				Allocator.Persistent
			);
			groundHitInfoArray = new NativeArray<RaycastHit>(maxVehicleCount, Allocator.Persistent);

			boxcastCommands = new NativeArray<BoxcastCommand>(
				maxVehicleCount * MaxBoxcastCount,
				Allocator.Persistent
			);
			obstacleHitInfoArray = new NativeArray<RaycastHit>(
				maxVehicleCount * MaxBoxcastCount,
				Allocator.Persistent
			);
			obstacleDistances = new NativeArray<float>(maxVehicleCount, Allocator.Persistent);

			isTurnings = new NativeArray<bool>(maxVehicleCount, Allocator.Persistent);
		}

		public void Dispose()
		{
			var dependsOn = JobHandle.CombineDependencies(
				raycastJobHandle,
				calculateObstacleDistanceJobHandle,
				curveCheckJobHandle
			);

			nativeStates.Dispose(dependsOn);
			waypoints.Dispose(dependsOn);

			raycastCommands.Dispose(dependsOn);
			groundHitInfoArray.Dispose(dependsOn);

			boxcastCommands.Dispose(dependsOn);
			obstacleHitInfoArray.Dispose(dependsOn);
			obstacleDistances.Dispose(dependsOn);

			isTurnings.Dispose(dependsOn);
		}

		public void Execute(IReadOnlyList<NPCVehicleInternalState> states, Transform egoTransform)
		{
			Profiler.BeginSample("Cognition.CheckNextWaypoint");

			new NextWaypointCheckJob { States = states }.Execute();

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

			obstacleCheckJobHandle = new ObstacleCheckJob
			{
				Commands = boxcastCommands,
				States = nativeStates,
				VehicleLayerMask = vehicleLayerMask,
				Waypoints = waypoints
			}.Schedule(boxcastCommands.Length, 16);

			boxcastJobHandle = BoxcastCommand.ScheduleBatch(
				boxcastCommands,
				obstacleHitInfoArray,
				16,
				obstacleCheckJobHandle
			);

			// Start background jobs
			JobHandle.ScheduleBatchedJobs();

			groundCheckJobHandle = new GroundCheckJob
			{
				Commands = raycastCommands,
				GroundLayerMask = groundLayerMask,
				NativeStates = nativeStates
			}.Schedule(nativeStates.Length, 8);

			raycastJobHandle = RaycastCommand.ScheduleBatch(
				raycastCommands,
				groundHitInfoArray,
				8,
				groundCheckJobHandle
			);

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

			new RightOfWayCheckJob { EGOTransform = egoTransform, States = states }.Execute();

			Profiler.EndSample();
			Profiler.BeginSample("Cognition.CheckTrafficLight");

			new TrafficLightCheckJob { States = states }.Execute();

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
				var stateCurrentPosition = state.FrontCenterPosition;
				stateCurrentPosition.y = state.Vehicle.transform.position.y + 2f;

				// Pose
				// if (state.Vehicle.VehicleID == 22 || state.Vehicle.VehicleID == 18)
				// {
				// 	Gizmos.color = Color.green;
				// 	Gizmos.DrawSphere(state.FrontCenterPosition, 1.2f);
				// 	Gizmos.color = Color.red;
				// 	Gizmos.DrawSphere(state.BackCenterPosition, 1.2f);
				// }

				// Start Stop
				// if (state.Vehicle.VehicleID == stateID || state.Vehicle.VehicleID == refID)
				// {
				// 	Vector3 PoseA = state.FrontCenterPosition;
				// 	Vector3? Goaln = state.BackCenterPosition;
				// 	if (Goaln != null)
				// 	{
				// 		Gizmos.color = Color.green;
				// 		Gizmos.DrawSphere(PoseA, 0.5f);
				// 		Gizmos.color = Color.red;
				// 		Gizmos.DrawSphere(Goaln.Value, 0.5f);
				// 	}
				// }

				// Dominating vehicle
				// if (state.Vehicle.VehicleID == refID && state.DominatingVehicle != null)
				// {
				//     Gizmos.color = Color.blue;
				//     Gizmos.DrawSphere(state.DominatingVehicle.transform.position, 3.2f);
				// }
				if (state.YieldPhase == NPCVehicleYieldPhase.NONE
					|| state.YieldPhase == NPCVehicleYieldPhase.ENTERING_INTERSECTION
					|| state.YieldPhase == NPCVehicleYieldPhase.AT_INTERSECTION)
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

			// Obstacle checking
			for (var stateIndex = 0; stateIndex < states.Count; stateIndex++)
			{
				Gizmos.color = states[stateIndex].IsStoppedByFrontVehicle
					? new Color(1f, 0.2f, 0f)
					: Color.cyan;

				var boxcastCount = Mathf.Min(
					MaxBoxcastCount,
					nativeStates[stateIndex].WaypointCount
				);
				for (
					var commandIndex = stateIndex * MaxBoxcastCount;
					commandIndex < stateIndex * MaxBoxcastCount + boxcastCount;
					commandIndex++
				)
				{
					var hitInfo = obstacleHitInfoArray[commandIndex];
					var hasHit = hitInfo.collider != null;

					var command = boxcastCommands[commandIndex];
					var startPoint = command.center;
					var direction = command.direction;
					var distance = hasHit ? hitInfo.distance : command.distance;
					var extents = command.halfExtents;
					var destination = startPoint + distance * direction;
					var rotation = Quaternion.LookRotation(direction);
					Gizmos.matrix = Matrix4x4.TRS(
						(destination + startPoint) / 2f,
						rotation,
						Vector3.one
					);
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
