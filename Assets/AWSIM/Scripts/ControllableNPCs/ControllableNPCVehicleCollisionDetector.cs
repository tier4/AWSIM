using System;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// Class used to detect when this object collides with other specified objects.
    /// </summary>
    public class ControllableNPCVehicleCollisionDetector : MonoBehaviour
    {
        public enum CollisionType
        {
            UNKNOWN,
            VEHICLE,
            ENVIRONMENT,
        }

        [Header("Settings")]
        [SerializeField] private LayerMask vehicleMask = default;
        [SerializeField] private LayerMask environmentMask = default;

        private Action<CollisionType> onCollisionDetected = default;

        public void Init(Action<CollisionType> collisionDetectedCallback)
        {
            this.onCollisionDetected = collisionDetectedCallback;
        }

        private void OnCollisionEnter(Collision other) 
        {
            if(((1 << other.gameObject.layer) & vehicleMask) != 0)
            {
                onCollisionDetected?.Invoke(CollisionType.VEHICLE);
            }
            else if(((1 << other.gameObject.layer) & environmentMask) != 0)
            {
                onCollisionDetected?.Invoke(CollisionType.ENVIRONMENT);
            }
            else
            {
                onCollisionDetected?.Invoke(CollisionType.UNKNOWN);
            }
        }
    }

}
