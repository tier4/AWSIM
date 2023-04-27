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
    }
}
