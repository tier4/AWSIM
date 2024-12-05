using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace AWSIM.TrafficSimulation
{
    public enum TrafficLightPassability
    {
        GREEN,
        YELLOW,
        RED
    }

    /// <summary>
    /// Utility class for RandomTraffic.
    /// </summary>
    public static class RandomTrafficUtils
    {
        /// <summary>
        /// Recognize signal information from bulb data.
        /// </summary>
        /// <param name="trafficLight">Traffic light to get bulb data</param>
        /// <param name="direction">Turning direction.</param>
        /// <returns>GREEN if the vehicle can pass. YELLOW if the vehicle should stop if it is safe to do so. RED if the vehicle should stop.</returns>
        public static TrafficLightPassability GetPassability(TrafficLight trafficLight, TrafficLane.TurnDirectionType direction = TrafficLane.TurnDirectionType.STRAIGHT)
        {
            var bulbDataArray = trafficLight.GetBulbData();

            var hasArrowSignal = false;
            var arrowDirection = TrafficLane.TurnDirectionType.STRAIGHT;
            var flashingColor = TrafficLightPassability.GREEN;
            foreach (var bulbData in bulbDataArray)
            {
                if (bulbData.Status == TrafficLight.BulbStatus.SOLID_OFF)
                    continue;

                switch (bulbData.Type)
                {
                    case TrafficLight.BulbType.RED_BULB:
                        flashingColor = TrafficLightPassability.RED;
                        break;
                    case TrafficLight.BulbType.YELLOW_BULB:
                        flashingColor = TrafficLightPassability.YELLOW;
                        break;
                    case TrafficLight.BulbType.GREEN_BULB:
                        flashingColor = TrafficLightPassability.GREEN;
                        break;
                    case TrafficLight.BulbType.LEFT_ARROW_BULB:
                        arrowDirection = TrafficLane.TurnDirectionType.LEFT;
                        hasArrowSignal = true;
                        break;
                    case TrafficLight.BulbType.RIGHT_ARROW_BULB:
                        arrowDirection = TrafficLane.TurnDirectionType.RIGHT;
                        hasArrowSignal = true;
                        break;
                    case TrafficLight.BulbType.UP_ARROW_BULB:
                        arrowDirection = TrafficLane.TurnDirectionType.STRAIGHT;
                        hasArrowSignal = true;
                        break;
                }
            }

            // TODO: Does it strictly meet specifications?
            if (hasArrowSignal)
                return arrowDirection != direction
                    ? TrafficLightPassability.RED
                    : TrafficLightPassability.GREEN;

            return flashingColor;
        }

        /// <summary>
        /// Get random element from <paramref name="source"/>
        /// </summary>
        /// <returns>Randomly selected element.</returns>
        public static T GetRandomElement<T>(IList<T> source)
        {
            return source.Any()
                ? source[Random.Range(0, source.Count)]
                : default;
        }
        
        /// <summary>
        /// Calculates where on the lane the position currently is.
        /// Helps to track progress of lane following functionality.
        /// </summary>
        /// <param name="position">position for which the progress is calculated</param>
        /// <param name="lane">lane on which the progress is calculated</param>
        /// <param name="progress">Out parameter. Represents distance from LaneStart to position projection on segment, relative to whole lane length [0-1 range]</param>
        /// <param name="laneLength">Out parameter. Represents</param>
        /// <returns></returns>
        public static void GetLaneFollowingProgressAndLaneLength(Vector3 position, TrafficLane lane, out float progress, out float laneLength)
        {
            if (lane is null)
            {
                progress = -1f;
                laneLength = -1f;
                return;
            }

            float lengthToPointOnLane = 0.0f;
            laneLength = 0.0f;
            float eps = 0.01f;
            for (var i = 0; i < lane.Waypoints.Length - 1; i++)
            {
                Vector3 segmentStart = lane.Waypoints[i];
                Vector3 segmentEnd = lane.Waypoints[i+1];
                Vector3 pointOnSegment = ClosestPointOnSegment(segmentStart, segmentEnd, position);
                float distanceFromStart = Vector3.Distance(segmentStart, pointOnSegment);
                if (distanceFromStart > eps)
                {
                    lengthToPointOnLane += distanceFromStart;
                }

                laneLength += Vector3.Distance(segmentStart, segmentEnd);
            }
            progress = lengthToPointOnLane / laneLength;
        }

        /// <summary>
        /// Calculates closest point on segment to point of interest.
        /// If point of interest cannot be projected to segment, returns closest end of segment.
        /// </summary>
        /// <param name="segmentStart">segment start position</param>
        /// <param name="segmentStop">segment end position</param>
        /// <param name="pointOfInterest">point of interest</param>
        /// <returns>closest point on segment to point of interest</returns>
        public static Vector3 ClosestPointOnSegment(Vector3 segmentStart, Vector3 segmentStop, Vector3 pointOfInterest)
        {
            Vector3 segmentVector = segmentStop - segmentStart;
            Vector3 segmentStartToPoiVector = pointOfInterest - segmentStart;

            float segmentMagnitude = segmentVector.sqrMagnitude;
            float dotProduct = Vector3.Dot(segmentStartToPoiVector, segmentVector);
            float argument = dotProduct / segmentMagnitude;

            float t = Mathf.Clamp01(argument);

            return segmentStart + t * segmentVector;
        }
    }
}
