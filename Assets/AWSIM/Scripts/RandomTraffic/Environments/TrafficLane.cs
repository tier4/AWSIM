using System.Collections.Generic;
using UnityEngine;

namespace AWSIM.TrafficSimulation
{
    /// <summary>
    /// Traffic lane component.
    /// </summary>
    public class TrafficLane : MonoBehaviour
    {
        /// <summary>
        /// Turning direction type of vehicles.
        /// </summary>
        public enum TurnDirectionType
        {
            STRAIGHT = 0,
            LEFT = 1,
            RIGHT = 2,
            NULL = 3
        }

        [SerializeField, Tooltip("Waypoints in this lane.")]
        private Vector3[] waypoints;
        [SerializeField, Tooltip("Turning direction of vehicles in the lane.")]
        private TurnDirectionType turnDirection;
        [SerializeField, Tooltip("Next lanes connected to this lane.")]
        private List<TrafficLane> nextLanes = new List<TrafficLane>();
        [SerializeField, Tooltip("Lanes leading to this lane.")]
        private List<TrafficLane> prevLanes = new List<TrafficLane>();
        [SerializeField, Tooltip("Lanes to which vehicles in this lane should yield the right of way.")]
        private List<TrafficLane> rightOfWayLanes = new List<TrafficLane>();
        [SerializeField, Tooltip("Stop line in the lane")]
        private StopLine stopLine;
        [SerializeField, Tooltip("Speed limit in m/s")]
        private float speedLimit;
        [SerializeField, Tooltip("Is intersection lane")]
        public bool intersectionLane;

        /// <summary>
        /// Get waypoints in this lane.
        /// </summary>
        public Vector3[] Waypoints => waypoints;

        public bool intersect(TrafficLane lane)
        {
            return IsPathIntersecting(waypoints, lane.waypoints);


            bool IsPathIntersecting(Vector3[] pathA, Vector3[] pathB)
            {
                for (int i = 0; i < pathA.Length - 1; i++)
                {
                    for (int j = 0; j < pathB.Length - 1; j++)
                    {
                        if (DoSegmentsIntersect(pathA[i], pathA[i + 1], pathB[j], pathB[j + 1]))
                        {
                            return true;
                        }
                    }
                }
                return false;
            }

            bool DoSegmentsIntersect(Vector3 p1, Vector3 p2, Vector3 q1, Vector3 q2)
            {
                float A1 = p2.y - p1.y;
                float B1 = p1.x - p2.x;
                float C1 = A1 * p1.x + B1 * p1.y;

                float A2 = q2.y - q1.y;
                float B2 = q1.x - q2.x;
                float C2 = A2 * q1.x + B2 * q1.y;

                float det = A1 * B2 - A2 * B1;

                if (det == 0)
                {
                    return false;
                }
                else
                {
                    float x = (B2 * C1 - B1 * C2) / det;
                    float y = (A1 * C2 - A2 * C1) / det;

                    return Mathf.Min(p1.x, p2.x) <= x && x <= Mathf.Max(p1.x, p2.x) &&
                           Mathf.Min(p1.y, p2.y) <= y && y <= Mathf.Max(p1.y, p2.y) &&
                           Mathf.Min(q1.x, q2.x) <= x && x <= Mathf.Max(q1.x, q2.x) &&
                           Mathf.Min(q1.y, q2.y) <= y && y <= Mathf.Max(q1.y, q2.y);
                }
            }
        }

        /// <summary>
        /// Get turning direction of vehicles in the lane.
        /// </summary>
        public TurnDirectionType TurnDirection => turnDirection;

        /// <summary>
        /// Get next lanes connected to this lane.
        /// </summary>
        public List<TrafficLane> NextLanes => nextLanes;

        /// <summary>
        /// Get lanes leading to this lane.
        /// </summary>
        public List<TrafficLane> PrevLanes => prevLanes;

        /// <summary>
        /// Get lanes to which vehicles in this lane should yield the right of way.
        /// </summary>
        public List<TrafficLane> RightOfWayLanes => rightOfWayLanes;

        /// <summary>
        /// Get a stop line in the lane.
        /// </summary>
        public StopLine StopLine
        {
            get => stopLine;
            set => stopLine = value;
        }

        /// <summary>
        /// Get speed limit in m/s.
        /// </summary>
        public float SpeedLimit => speedLimit;

        /// <summary>
        /// Create <see cref="TrafficLane"/> instance in the scene.<br/>
        /// </summary>
        /// <param name="wayPoints"></param>
        /// <param name="speedLimit"></param>
        /// <returns><see cref="TrafficLane"/> instance.</returns>
        public static TrafficLane Create(Vector3[] wayPoints, TurnDirectionType turnDirection, float speedLimit = 0f)
        {
            var gameObject = new GameObject("TrafficLane", typeof(TrafficLane));
            gameObject.transform.position = wayPoints[0];
            var trafficLane = gameObject.GetComponent<TrafficLane>();
            trafficLane.waypoints = wayPoints;
            trafficLane.turnDirection = turnDirection;
            trafficLane.speedLimit = speedLimit;
            return trafficLane;
        }
    }
}
