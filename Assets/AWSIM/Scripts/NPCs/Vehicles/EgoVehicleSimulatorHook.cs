using UnityEngine;

namespace AWSIM.TrafficSimulation
{

/// <summary>
/// Sets this GameObject as Ego for vehicle simulator component.
/// </summary>
public class EgoVehicleSimulatorHook : MonoBehaviour
{
    TrafficManager trafficManager;

    void Start()
    {
        trafficManager = FindObjectOfType<TrafficManager>();
        trafficManager.egoVehicle = this.gameObject;
    }

    void OnDestroy()
    {
        trafficManager.egoVehicle = null;
    }
}
}
