using UnityEngine;

namespace AWSIM.TrafficSimulation
{

/// <summary>
/// Test component for spawning RouteTrafficSimulator 
/// </summary>
public class SpawnerTest : MonoBehaviour
{
    public TrafficManager tm;
    public TrafficLane[] routeLanes;
    public GameObject[] vehiclePrefabs;

    void Update()
    {
        if (Input.GetKeyUp(KeyCode.Space))
        {
            var rts = new RouteTrafficSimulator(
                this.gameObject,
                vehiclePrefabs,
                routeLanes,
                tm.npcVehicleSimulator,
                1
            );
            tm.AddTrafficSimulator(rts);
        }
    }
}
}
