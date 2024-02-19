using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// By attaching this script to a collider whose IsTrigger is On, the slip rate of the Vehicle can be changed. 1 no slip, 0 is full slip.
    /// </summary>
    public class GroudSlipMultiplier : MonoBehaviour
    {
        [Range(0, 1)] float forwardSlip;
        [Range(0, 1)] float sidewaySlip;

        /// <summary>
        /// Slip multiplier in foward direction.
        /// </summary>
        public float FowardSlip => forwardSlip;

        /// <summary>
        /// Slip multiplier in sideway direction.
        /// </summary>
        public float SidewaySlip => sidewaySlip;
    }
}