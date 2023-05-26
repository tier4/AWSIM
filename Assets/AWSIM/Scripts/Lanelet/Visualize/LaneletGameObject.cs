using UnityEngine;

// TODO: think of a better place to put this
// TODO: think of a better name

namespace AWSIM.Lanelet
{
    public class LaneletGameObject : MonoBehaviour
    {
        [SerializeField, Tooltip("Waypoints in this lane.")]
        private Vector3[] waypoints;

        public Vector3[] Waypoints => waypoints;

        /// <summary>
        /// Create <see cref="LaneletGameObject"/> instance in the scene.
        /// </summary>
        /// <param name="wayPoints"></param>
        /// <returns><see cref="LaneletGameObject"/> instance.</returns>
        public static LaneletGameObject Create(Vector3[] wayPoints)
        {
            var gameObject = new GameObject("LaneletBound", typeof(LaneletGameObject));
            gameObject.transform.position = wayPoints[0];
            var laneletBound = gameObject.GetComponent<LaneletGameObject>();
            laneletBound.waypoints = wayPoints;
            return laneletBound;
        }
    }
}
