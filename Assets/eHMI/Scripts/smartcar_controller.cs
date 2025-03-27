// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;

// namespace AWSIM
// {
//     /// <summary>
//     /// NPC Vehicle class that follows waypoints.
//     /// </summary>
//     public class smartcar_controller : MonoBehaviour
//     {
//         [Header("Physics Settings")]
//         [SerializeField] Transform centerOfMass;
//         [SerializeField] new Rigidbody rigidbody;

//         [Header("Waypoint Settings")]
//         [SerializeField] List<Transform> waypoints; // List of waypoints the vehicle will follow
//         [SerializeField] float waypointReachThreshold = 1f; // Distance to waypoint before moving to next

//         private int currentWaypointIndex = 0; // Current waypoint the vehicle is targeting

//         // Movement parameters
//         [SerializeField] float moveSpeed = 5f;  // Speed of the vehicle

//         // Start is called before the first frame update
//         void Awake()
//         {
//             if (rigidbody == null)
//                 rigidbody = GetComponent<Rigidbody>();

//             rigidbody.centerOfMass = transform.InverseTransformPoint(centerOfMass.position);
//         }

//         // Update is called once per frame
//         void Update()
//         {
//             if (waypoints.Count == 0) return;

//             // Move the car towards the current waypoint
//             MoveTowardsWaypoint();
//         }

//         void MoveTowardsWaypoint()
//         {
//             // Get the current target waypoint
//             Transform targetWaypoint = waypoints[currentWaypointIndex];

//             // Calculate direction to the target waypoint
//             Vector3 direction = (targetWaypoint.position - transform.position).normalized;

//             // Move the car towards the waypoint (using Rigidbody for physics-based movement)
//             rigidbody.velocity = direction * moveSpeed;

//             // Check if the vehicle is close enough to the waypoint
//             if (Vector3.Distance(transform.position, targetWaypoint.position) < waypointReachThreshold)
//             {
//                 // Move to the next waypoint, wrapping around if necessary
//                 currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Count;
//             }
//         }

//         private void OnValidate()
//         {
//             if (rigidbody == null)
//                 rigidbody = GetComponent<Rigidbody>();
//         }

//         private void OnDrawGizmos()
//         {
//             // Draw waypoints as gizmos for visualization in the editor
//             if (waypoints != null)
//             {
//                 foreach (Transform waypoint in waypoints)
//                 {
//                     Gizmos.color = Color.red;
//                     Gizmos.DrawSphere(waypoint.position, 0.5f);
//                 }
//             }
//         }
//     }
// }

// using UnityEngine;

// public class smartcar_controller : MonoBehaviour
// {
//     public Transform[] waypoints; // Array to hold the waypoints
//     public float speed = 5f; // Movement speed
//     private int currentWaypointIndex = 0; // To track the current waypoint
//     public float waypointThreshold = 0.5f; // Distance threshold to switch to the next waypoint

//     void Update()
//     {
//         // If there are waypoints and we're not at the last one
//         if (waypoints.Length > 0 && currentWaypointIndex < waypoints.Length)
//         {
//             // Move towards the current waypoint
//             Transform targetWaypoint = waypoints[currentWaypointIndex];
//             Vector3 direction = (targetWaypoint.position - transform.position).normalized;
//             transform.position += direction * speed * Time.deltaTime;

//             // If we are close enough to the waypoint, move to the next one
//             if (Vector3.Distance(transform.position, targetWaypoint.position) < waypointThreshold)
//             {
//                 currentWaypointIndex++; // Move to the next waypoint
//             }
//         }
//     }
// }


// using UnityEngine;
// using System.Collections;

// public class smartcar_controller : MonoBehaviour
// {
//     public Transform[] waypoints; // Array to hold the waypoints
//     public float speed = 2f; // Movement speed
//     private int currentWaypointIndex = 0; // To track the current waypoint
//     public float waypointThreshold = 0.5f; // Distance threshold to switch to the next waypoint
//     private bool isWaiting = false; // To check if the car is waiting

//     void Update()
//     {
//         // If there are waypoints, we're not at the last one, and not currently waiting
//         if (waypoints.Length > 0 && currentWaypointIndex < waypoints.Length && !isWaiting)
//         {
//             // Get the target waypoint
//             Transform targetWaypoint = waypoints[currentWaypointIndex];

//             // Ignore Y-axis to keep movement on a flat plane
//             Vector3 targetPosition = new Vector3(targetWaypoint.position.x, transform.position.y, targetWaypoint.position.z);
//             Vector3 direction = (targetPosition - transform.position).normalized;

//             // Move towards the waypoint
//             transform.position += direction * speed * Time.deltaTime;

//             // Smoothly rotate the car to face the direction of movement
//             if (direction.magnitude > 0.1f) // Ensure we have a valid direction
//             {
//                 Quaternion targetRotation = Quaternion.LookRotation(direction);
//                 transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, Time.deltaTime * 5f);
//             }

//             // If we are close enough to the waypoint, handle pause at the first waypoint
//             if (Vector3.Distance(transform.position, targetWaypoint.position) < waypointThreshold)
//             {
//                 if (currentWaypointIndex == 0) // Pause only at the first waypoint
//                 {
//                     StartCoroutine(WaitAtFirstWaypoint(10f)); // Wait for 10 seconds
//                 }
//                 else
//                 {
//                     currentWaypointIndex++; // Move to the next waypoint immediately
//                 }
//             }
//         }

//     }

//     IEnumerator WaitAtFirstWaypoint(float waitTime)
//     {
//         isWaiting = true; // Set waiting to true
//         yield return new WaitForSeconds(waitTime); // Wait for the specified time
//         currentWaypointIndex++; // Move to the next waypoint
//         isWaiting = false; // Set waiting back to false
//     }
// }


using UnityEngine;
using System.Collections;

public class smartcar_controller : MonoBehaviour
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
                    StartCoroutine(WaitAtFirstWaypoint(3f)); // Wait for 10 seconds
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
