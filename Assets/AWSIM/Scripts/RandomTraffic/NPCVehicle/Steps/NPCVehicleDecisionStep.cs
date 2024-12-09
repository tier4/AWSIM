using System.Collections.Generic;
using UnityEngine;

namespace AWSIM.TrafficSimulation
{
    /// <summary>
    /// Decision step implementation for a NPC vehicle simulation.
    /// Based on the results of the cognitive step, it outputs a short-term target point and a decision to decelerate or accelerate.
    /// </summary>
    public class NPCVehicleDecisionStep
    {
        private NPCVehicleConfig config;

        // MinFrontVehicleDistance is added to the threshold for the distance at which an obstacle is considered dangerous.
        // The vehicle is controlled to stop at this distance away from the obstacle(e.g. another vehicle in front of the vehicle).
        private const float MinFrontVehicleDistance = 4f;
        private const float MinStopDistance = 1.5f;

        public NPCVehicleDecisionStep(NPCVehicleConfig config)
        {
            this.config = config;
        }

        public void Execute(IReadOnlyList<NPCVehicleInternalState> states)
        {
            foreach (var state in states)
            {
                UpdateTargetPoint(state);
                UpdateSpeedMode(state, config);
            }
        }

        /// <summary>
        /// Set short-term target point of the vehicle to the next waypoint.
        /// </summary>
        /// <param name="state"></param>
        private static void UpdateTargetPoint(NPCVehicleInternalState state)
        {
            if (state.ShouldDespawn || state.CurrentFollowingLane == null)
                return;

            state.TargetPoint = state.CurrentFollowingLane.Waypoints[state.WaypointIndex];
        }

        /// <summary>
        /// Update speed mode according to the following cognition results and stoppable distance calculated by speed.<br/>
        /// Possible speed modes and conditions are the following:<br/>
        /// - SLOW when the vehicle is in a sharp curve, needs to keep distance from a front vehicle or is entering yielding lane.<br/>
        /// - STOP when the vehicle can stop safely at a stop point(e.g. a stop line or a point that an obstacle exists).<br/>
        /// - SUDDEN_STOP when the vehicle cannot stop safely at a stop point.<br/>
        /// - ABSOLUTE_STOP when the vehicle cannot stop using SUDDEN_STOP<br/>
        /// - NORMAL under other conditions.
        // /// </summary>
        private static void UpdateSpeedMode(NPCVehicleInternalState state, NPCVehicleConfig config)
        {
            if (state.ShouldDespawn)
            {
                return;
            }

            var absoluteStopDistance = CalculateStoppableDistance(state.Speed, config.AbsoluteDeceleration) + MinStopDistance;
            var suddenStopDistance = CalculateStoppableDistance(state.Speed, config.SuddenDeceleration) + 2 * MinStopDistance;
            var stopDistance = CalculateStoppableDistance(state.Speed, config.Deceleration) + 3 * MinStopDistance;
            var slowDownDistance = stopDistance + 4 * MinStopDistance;

            var distanceToStopPointByFrontVehicle = onlyGreaterThan(state.DistanceToFrontVehicle - MinFrontVehicleDistance, -MinFrontVehicleDistance);
            var distanceToStopPointByTrafficLight = CalculateTrafficLightDistance(state, suddenStopDistance);
            var distanceToStopPointByRightOfWay = CalculateYieldingDistance(state);
            var distanceToStopPoint = Mathf.Min(distanceToStopPointByFrontVehicle, distanceToStopPointByTrafficLight, distanceToStopPointByRightOfWay);

            state.IsStoppedByFrontVehicle = false;
            if (distanceToStopPointByFrontVehicle <= stopDistance)
            {
                state.IsStoppedByFrontVehicle = true;
            }

            if (distanceToStopPoint <= absoluteStopDistance)
                state.SpeedMode = NPCVehicleSpeedMode.ABSOLUTE_STOP;
            else if (distanceToStopPoint <= suddenStopDistance)
                state.SpeedMode = NPCVehicleSpeedMode.SUDDEN_STOP;
            else if (distanceToStopPoint <= stopDistance)
                state.SpeedMode = NPCVehicleSpeedMode.STOP;
            else if (distanceToStopPoint <= slowDownDistance || state.IsTurning)
                state.SpeedMode = NPCVehicleSpeedMode.SLOW;
            else
                state.SpeedMode = NPCVehicleSpeedMode.NORMAL;
        }

        private static float CalculateTrafficLightDistance(NPCVehicleInternalState state, float suddenStopDistance)
        {
            var distanceToStopPointByTrafficLight = float.MaxValue;
            if (state.TrafficLightLane != null)
            {
                var distanceToStopLine = DistanceToClosestTrafficLane(state);
                switch (state.TrafficLightPassability)
                {
                    case TrafficLightPassability.GREEN:
                        break;
                    case TrafficLightPassability.YELLOW:
                        if (distanceToStopLine < suddenStopDistance) break;
                        distanceToStopPointByTrafficLight = distanceToStopLine;
                        break;
                    case TrafficLightPassability.RED:
                        distanceToStopPointByTrafficLight = distanceToStopLine;
                        break;
                }
            }
            return onlyGreaterThan(distanceToStopPointByTrafficLight, 0);
        }

        private static float DistanceToClosestTrafficLane(NPCVehicleInternalState state)
        {
            if (state.TrafficLightLane is null && !state.FollowingLanes.Contains(state.TrafficLightLane))
            {
                return float.MaxValue;
            }

            var distance = 0f;
            var vehiclePosition = state.FrontCenterPosition;
            bool startAddingWholeLanesDistance = false;

            for (var i = 0; i < state.FollowingLanes.Count; i++)
            {
                (var laneFollowingProgress, var laneLenght) =
                    GetLaneFollowingProgressAndLaneLength(vehiclePosition, state.FollowingLanes[i]);
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
                if (state.FollowingLanes[i] == state.TrafficLightLane)
                {
                    break;
                }
            }
            return distance;
        }

        public static (float, float) GetLaneFollowingProgressAndLaneLength(Vector3 position, TrafficLane lane)
        {
            if (lane is null)
            {
                return (-1f, -1f);
            }

            float lengthToPointOnLane = 0.0f;
            float wholeLaneLength = 0.0f;
            float eps = 0.01f;
            Vector2 positionXZ = new Vector2(position.x, position.z);
            for (var i = 0; i < lane.Waypoints.Length - 1; i++)
            {
                Vector2 segmentStart = new Vector2(lane.Waypoints[i].x, lane.Waypoints[i].z);
                Vector2 segmentEnd = new Vector2(lane.Waypoints[i + 1].x, lane.Waypoints[i + 1].z);
                Vector2 pointOnSegment = ClosestPointOnSegment(segmentStart, segmentEnd, positionXZ);
                float distanceFromStart = Vector2.Distance(segmentStart, pointOnSegment);
                if (distanceFromStart > eps)
                {
                    lengthToPointOnLane += distanceFromStart;
                }

                wholeLaneLength += Vector2.Distance(segmentStart, segmentEnd);
            }

            return ((lengthToPointOnLane / wholeLaneLength), wholeLaneLength);
        }

        public static Vector2 ClosestPointOnSegment(Vector2 segmentStart, Vector2 segmentStop, Vector2 pointOfInterest)
        {
            Vector2 segmentVector = segmentStop - segmentStart;
            Vector2 segmentStartToPoiVector = pointOfInterest - segmentStart;

            float segmentMagnitude = segmentVector.sqrMagnitude;
            float dotProduct = Vector2.Dot(segmentStartToPoiVector, segmentVector);
            float argument = dotProduct / segmentMagnitude;

            float t = Mathf.Clamp01(argument);

            return segmentStart + t * segmentVector;
        }

        private static float CalculateYieldingDistance(NPCVehicleInternalState state)
        {
            var distanceToStopPointByRightOfWay = float.MaxValue;
            if (state.YieldPhase != NPCVehicleYieldPhase.NONE && state.YieldPhase != NPCVehicleYieldPhase.ENTERING_INTERSECTION && state.YieldPhase != NPCVehicleYieldPhase.AT_INTERSECTION)
                distanceToStopPointByRightOfWay = state.SignedDistanceToPointOnLane(state.YieldPoint);
            return onlyGreaterThan(distanceToStopPointByRightOfWay, -float.MaxValue);
        }

        private static float CalculateStoppableDistance(float speed, float deceleration)
        {
            return onlyGreaterThan(speed * speed / 2f / deceleration, 0);
        }

        private static float onlyGreaterThan(float value, float min_value = 0)
        { return value >= min_value ? value : float.MaxValue; }

        public void ShowGizmos(IReadOnlyList<NPCVehicleInternalState> states)
        {
            foreach (var state in states)
            {
                switch (state.SpeedMode)
                {
                    case NPCVehicleSpeedMode.ABSOLUTE_STOP:
                    case NPCVehicleSpeedMode.SUDDEN_STOP:
                    case NPCVehicleSpeedMode.STOP:
                        Gizmos.color = Color.red;
                        break;
                    case NPCVehicleSpeedMode.SLOW:
                        Gizmos.color = Color.yellow;
                        break;
                    default:
                        Gizmos.color = Color.green;
                        break;
                }

                var currentPosition = state.FrontCenterPosition;
                currentPosition.y += 1f;

                Gizmos.DrawLine(currentPosition, state.TargetPoint);

                var rotation = Quaternion.LookRotation(currentPosition - state.TargetPoint);
                Gizmos.matrix = Matrix4x4.TRS(state.TargetPoint, rotation, Vector3.one);
                Gizmos.DrawFrustum(Vector3.zero, 30f, 1f, 0f, 1f);
                Gizmos.matrix = Matrix4x4.identity;
            }
        }
    }
}