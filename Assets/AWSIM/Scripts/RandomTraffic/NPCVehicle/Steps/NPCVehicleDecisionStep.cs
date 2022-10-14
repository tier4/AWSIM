using System.Collections.Generic;
using UnityEngine;

namespace AWSIM.RandomTraffic
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
        /// - NORMAL under other conditions.
        /// </summary>
        private static void UpdateSpeedMode(NPCVehicleInternalState state, NPCVehicleConfig config)
        {
            if (state.ShouldDespawn)
                return;

            var maxStoppableDistance =
                Mathf.Max(CalculateStoppableDistance(state.Speed, config.Deceleration), 3f);
            var minStoppableDistance =
                Mathf.Max(CalculateStoppableDistance(state.Speed, config.SuddenDeceleration), 3f);

            var distanceToStopPointByFrontVehicle = state.DistanceToFrontVehicle - MinFrontVehicleDistance;

            // Speed mode updated by front vehicle is SLOW or SUDDEN_STOP.
            // STOP state is not used to prevent the state from changing every frame.
            if (distanceToStopPointByFrontVehicle <= minStoppableDistance)
            {
                state.IsStoppedByFrontVehicle = true;
                state.SpeedMode = NPCVehicleSpeedMode.SUDDEN_STOP;
                return;
            }
            state.IsStoppedByFrontVehicle = false;

            var distanceToStopPointByTrafficLight = float.MaxValue;
            if (state.TrafficLightLane != null)
            {
                var distanceToStopLine =
                    state.SignedDistanceToPointOnLane(state.TrafficLightLane.StopLine.CenterPoint);
                switch (state.TrafficLightPassability)
                {
                    case TrafficLightPassability.GREEN:
                        break;
                    case TrafficLightPassability.YELLOW:
                        var canStopSafely = distanceToStopLine >= minStoppableDistance;
                        if (!canStopSafely)
                            break;
                        distanceToStopPointByTrafficLight = distanceToStopLine;
                        break;
                    case TrafficLightPassability.RED:
                        distanceToStopPointByTrafficLight = distanceToStopLine;
                        break;
                }
            }

            var distanceToStopPointByRightOfWay = float.MaxValue;
            if (state.YieldPhase == NPCVehicleYieldPhase.YIELDING)
            {
                distanceToStopPointByRightOfWay = state.SignedDistanceToPointOnLane(state.YieldPoint);
            }

            var distanceToStopPoint = Mathf.Min(
                distanceToStopPointByTrafficLight,
                distanceToStopPointByRightOfWay);

            // Sudden stop if the vehicle cannot stop safely.
            var shouldStopSuddenly =
                distanceToStopPoint <= minStoppableDistance ||
                state.YieldPhase == NPCVehicleYieldPhase.YIELDING;
            if (shouldStopSuddenly)
            {
                state.SpeedMode = NPCVehicleSpeedMode.SUDDEN_STOP;
                return;
            }

            // Stop normally when the vehicle is reaching stop point.
            var shouldStopNormally =
                distanceToStopPoint <= maxStoppableDistance;
            if (shouldStopNormally)
            {
                state.SpeedMode = NPCVehicleSpeedMode.STOP;
                return;
            }

            var shouldSlowDownByRightOfWay =
                state.YieldPhase == NPCVehicleYieldPhase.ENTERING_YIELDING_LANE;

            var shouldSlowDown =
                // Should slow down by front vehicle (for keeping distance)
                state.DistanceToFrontVehicle <= maxStoppableDistance + MinFrontVehicleDistance ||
                shouldSlowDownByRightOfWay ||
                state.IsTurning;
            if (shouldSlowDown)
            {
                state.SpeedMode = NPCVehicleSpeedMode.SLOW;
                return;
            }

            state.SpeedMode = NPCVehicleSpeedMode.NORMAL;
        }

        private static float CalculateStoppableDistance(float speed, float deceleration)
        {
            return speed * speed / 2f / deceleration;
        }

        public void ShowGizmos(IReadOnlyList<NPCVehicleInternalState> states)
        {
            foreach (var state in states)
            {
                switch (state.SpeedMode)
                {
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
                currentPosition.y = state.Vehicle.transform.position.y + 1f;

                Gizmos.DrawLine(currentPosition, state.TargetPoint);

                var rotation = Quaternion.LookRotation(currentPosition - state.TargetPoint);
                Gizmos.matrix = Matrix4x4.TRS(state.TargetPoint, rotation, Vector3.one);
                Gizmos.DrawFrustum(Vector3.zero, 30f, 1f, 0f, 1f);
                Gizmos.matrix = Matrix4x4.identity;
            }
        }
    }
}
