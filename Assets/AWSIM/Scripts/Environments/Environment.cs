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

        /// <summary>
        /// Reference point of Autoware's MGRS coordinate system.
        /// </summary>
        public Vector3 MgrsOffsetPosition => mgrsOffsetPosition;
    }
}