using UnityEngine;

public class CarSpawner : MonoBehaviour
{
    [Header("Car Settings")]
    [SerializeField] private GameObject carPrefab; // Drag your car prefab here in the Inspector
    [SerializeField] private Transform spawnPoint; // Drag your spawn point (empty GameObject) here in the Inspector
    [SerializeField] private Transform[] waypoints; // Drag your waypoints here

    private GameObject spawnedCar;

    /// <summary>
    /// Spawns a car at the designated spawn point.
    /// </summary>
    public void SpawnCar()
    {
        if (carPrefab == null)
        {
            Debug.LogError("Car prefab not assigned in the CarSpawner script.");
            return;
        }

        if (spawnPoint == null)
        {
            Debug.LogError("Spawn point not assigned in the CarSpawner script.");
            return;
        }

        if (spawnedCar != null)
        {
            Debug.LogWarning("A car is already spawned. Despawn the current car before spawning a new one.");
            return;
        }

        // Instantiate the car at the spawn point
        spawnedCar = Instantiate(carPrefab, spawnPoint.position, spawnPoint.rotation);

        // Optionally, align the car's initial rotation with the road's direction
        AlignCarToRoad();

        // Optionally, add initialization logic for the car
        Debug.Log("Car spawned successfully!");
    }

    private void AlignCarToRoad()
    {
        // Example: Assume you have a "road" direction stored in the spawnPoint
        // or you can get the road direction from another script or road object.
        Vector3 roadDirection = spawnPoint.forward;  // Assuming spawnPoint's forward direction is the road's direction

        // Set the car's rotation to match the road direction
        spawnedCar.transform.rotation = Quaternion.LookRotation(roadDirection);
    }

    
    private void OnDrawGizmos()
    {
        if (waypoints == null || waypoints.Length == 0) return;

        Gizmos.color = Color.red;
        for (int i = 0; i < waypoints.Length; i++)
        {
            if (waypoints[i] != null)
            {
                Gizmos.DrawSphere(waypoints[i].position, 0.5f);
                if (i < waypoints.Length - 1 && waypoints[i + 1] != null)
                {
                    Gizmos.DrawLine(waypoints[i].position, waypoints[i + 1].position);
                }
            }
        }
    }

    private void Start()
    {
        SpawnCar(); // This will automatically spawn the car when the game starts
    }

    /// <summary>
    /// Despawns the currently spawned car.
    /// </summary>
    public void DespawnCar()
    {
        if (spawnedCar != null)
        {
            Destroy(spawnedCar);
            spawnedCar = null;
            Debug.Log("Car despawned successfully!");
        }
        else
        {
            Debug.LogWarning("No car to despawn.");
        }
    }

    /// <summary>
    /// Sets a new spawn point dynamically (optional).
    /// </summary>
    /// <param name="newSpawnPoint">The new spawn point transform.</param>
    public void SetSpawnPoint(Transform newSpawnPoint)
    {
        if (newSpawnPoint != null)
        {
            spawnPoint = newSpawnPoint;
            Debug.Log("Spawn point updated successfully!");
        }
        else
        {
            Debug.LogError("Attempted to set a null spawn point.");
        }
    }
}
