using UnityEngine;

namespace AWSIM.TrafficSimulation
{

/// <summary>
/// Interface for traffic simulators
/// </summary>  
public interface ITrafficSimulator
{
    /// <summary>
    /// Try to spawn a prefab in a given spawn location. 
    /// </summary>
    /// <param name="prefab"></param>
    /// <param name="spawnPoint"></param>
    /// <param name="spawnedVehicle"></param>
    /// <returns>True if spawned, False is failed to spawn.</returns>
    public bool Spawn(GameObject prefab, NPCVehicleSpawnPoint spawnPoint, out NPCVehicle spawnedVehicle);
    
    /// <summary>
    /// Gets random spawn point and NPC prefab.
    /// </summary>
    public void GetRandomSpawnInfo(out NPCVehicleSpawnPoint spawnPoint, out GameObject prefab);

    /// <summary>
    /// Increase the traffic simulator spawn priority.
    /// <param name="spawnPoint"></param>
    /// </summary>
    public void IncreasePriority(int priority);

    /// <summary>
    /// Get current traffic simulator spawn priority.
    /// <returns>Priority value</returns>
    /// </summary>
    public int GetCurrentPriority();

    /// <summary>
    /// Reset traffic simulator spawn priority.
    /// </summary>
    public void ResetPriority();

    /// <summary>
    /// Is the traffic simulator enabled.
    /// </summary>
    public bool IsEnabled();
}
}
