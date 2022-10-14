using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace AWSIM
{
    /// <summary>
    /// Prevent inspector input for WheelCollider.
    /// WheelCollider friction is set to 0. Enable suspension and wheel collision only.
    /// </summary>
    [RequireComponent(typeof(WheelCollider))]
    public class WheelColliderConfig : MonoBehaviour
    {
        public WheelCollider WheelCollider => wheelCollider;
        [SerializeField, HideInInspector] WheelCollider wheelCollider;

        void Reset()
        {
            // Get WheelCollider component.
            wheelCollider = GetComponent<WheelCollider>();

            // Set WheelCollider intial values.
            wheelCollider.mass = 1f;
            wheelCollider.radius = 0.35f;
            wheelCollider.wheelDampingRate = 0.25f;
            wheelCollider.suspensionDistance = 0.2f;
            wheelCollider.forceAppPointDistance = 0f;
            wheelCollider.center = Vector3.zero;
            wheelCollider.suspensionSpring = new JointSpring()
            {
                spring = 35000f,
                damper = 3500f,
                targetPosition = 0.5f,
            };
            var fc = new WheelFrictionCurve()
            {
                extremumSlip = 0f,
                extremumValue = 0f,
                asymptoteSlip = 0f,
                asymptoteValue = 0f,
                stiffness = 0f,
            };
            wheelCollider.sidewaysFriction = fc;        // Set friction to 0.
            wheelCollider.forwardFriction = fc;         // Set friction to 0.
            wheelCollider.mass = 1.0f;
            wheelCollider.enabled = this.enabled;
        }
    }
}