using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// The Environment class.
    /// A collection of environment-related functions and property.
    /// </summary>
    public class Environment : SingletonMonoBehaviour<Environment>
    {
        [Tooltip("Enter in the MGRS coordinate system.")]
        [SerializeField]
        Vector3 mgrsOffsetPosition;

        [SerializeField]
        string mgrsGridZone;

        /// <summary>
        /// Reference point of MGRS coordinate system.
        /// </summary>
        public Vector3 MgrsOffsetPosition => mgrsOffsetPosition;

        /// <summary>
        /// MGRS Grid Zone.
        /// </summary>
        public string MgrsGridZone => mgrsGridZone;
    }
}