using UnityEngine;

namespace AWSIM.TrafficSimulation
{
    /// <summary>
    /// Stop line component.
    /// </summary>
    public class StopLine : MonoBehaviour
    {
        [SerializeField, Tooltip("Line data consists of 2 points.")]
        private Vector3[] points = new Vector3[2];
        [SerializeField, Tooltip("Indicates whether the stop sign exists.")] 
        private bool hasStopSign = false;
        [SerializeField, Tooltip("Traffic light ")]
        private TrafficLight trafficLight;

        /// <summary>
        /// Get line data consists of 2 points.
        /// </summary>
        public Vector3[] Points => points;

        /// <summary>
        /// Get center point of the stop line.
        /// </summary>
        public Vector3 CenterPoint => (points[0] + points[1]) / 2f;

        public TrafficLight TrafficLight
        {
            get => trafficLight;
            set => trafficLight = value;
        }

        public bool HasStopSign
        {
            get => hasStopSign;
            set => hasStopSign = value;
        }

        public static StopLine Create(Vector3 p1, Vector3 p2)
        {
            var gameObject = new GameObject("StopLine", typeof(StopLine));
            gameObject.transform.position = p1;
            var stopLine = gameObject.GetComponent<StopLine>();
            stopLine.points[0] = p1;
            stopLine.points[1] = p2;
            return stopLine;
        }
    }
}
