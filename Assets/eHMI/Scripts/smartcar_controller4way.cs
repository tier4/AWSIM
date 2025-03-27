using UnityEngine;

public class smartcar_controller4way : MonoBehaviour
{
    public Transform[] waypoints; // Array to hold the waypoints
    public float speed = 5f; // Movement speed
    private int currentWaypointIndex = 0; // To track the current waypoint
    public float waypointThreshold = 0.5f; // Distance threshold to switch to the next waypoint

    void Update()
    {
        // If there are waypoints and we're not at the last one
        if (waypoints.Length > 0 && currentWaypointIndex < waypoints.Length)
        {
            // Move towards the current waypoint
            Transform targetWaypoint = waypoints[currentWaypointIndex];
            Vector3 direction = (targetWaypoint.position - transform.position).normalized;
            transform.position += direction * speed * Time.deltaTime;

            // If we are close enough to the waypoint, move to the next one
            if (Vector3.Distance(transform.position, targetWaypoint.position) < waypointThreshold)
            {
                currentWaypointIndex++; // Move to the next waypoint
            }
        }
    }
}
