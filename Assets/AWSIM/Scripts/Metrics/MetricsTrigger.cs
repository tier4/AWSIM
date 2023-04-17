using UnityEngine;
using System;

namespace AWSIM
{
    /// <summary>
    /// Metrics trigger.
    /// Detects collision events of controllable objects. 
    /// </summary>
    public class MetricsTrigger : MonoBehaviour
    {
        public event Action<Vehicle, Collision> onCollision;
        Vehicle attachedVehicle;

        void Start()
        {
            attachedVehicle = GetComponent<Vehicle>();
            if (attachedVehicle == null)
            {
                Debug.LogError("Metrics trigger attached to a GameObject which is not a controllable vehicle.");
            }
        }

        void OnCollisionEnter(Collision collision)
        {
            if (attachedVehicle)
            {
                onCollision(attachedVehicle, collision);
            }
        }
    }
    
}
