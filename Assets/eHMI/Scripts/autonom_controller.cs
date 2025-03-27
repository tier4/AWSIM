using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class autonom_controller : MonoBehaviour
{
    public Transform[] waypoints; // Array to hold the waypoints
    public float speed = 2f; // Movement speed
    private int currentWaypointIndex = 0; // To track the current waypoint
    public float waypointThreshold = 0.5f; // Distance threshold to switch to the next waypoint
    private bool isWaiting = false; // To check if the car is waiting
    void Update()
    {
        // If there are waypoints, we're not at the last one, and not currently waiting
        if (waypoints.Length > 0 && currentWaypointIndex < waypoints.Length && !isWaiting)
        {
            // Get the target waypoint
            Transform targetWaypoint = waypoints[currentWaypointIndex];

            // Ignore Y-axis to keep movement on a flat plane
            Vector3 targetPosition = new Vector3(targetWaypoint.position.x, transform.position.y, targetWaypoint.position.z);
            Vector3 direction = (targetPosition - transform.position).normalized;

            // Move towards the waypoint
            transform.position += direction * speed * Time.deltaTime;

            // Smoothly rotate the car to face the direction of movement
            if (direction.magnitude > 0.1f) // Ensure we have a valid direction
            {
                Quaternion targetRotation = Quaternion.LookRotation(direction);
                transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, Time.deltaTime * 5f);
            }

            // If we are close enough to the waypoint, handle pause at the first waypoint
            if (Vector3.Distance(transform.position, targetWaypoint.position) < waypointThreshold)
            {
                if (currentWaypointIndex == 0) // Pause only at the first waypoint
                {
                    StartCoroutine(WaitAtFirstWaypoint(0f)); // Wait for 10 seconds
                }
                else
                {
                    currentWaypointIndex++; // Move to the next waypoint immediately
                }
            }
        }


    }

    IEnumerator WaitAtFirstWaypoint(float waitTime)
    {
        isWaiting = true; // Set waiting to true
        yield return new WaitForSeconds(waitTime); // Wait for the specified time
        currentWaypointIndex++; // Move to the next waypoint
        isWaiting = false; // Set waiting back to false
    }



}
