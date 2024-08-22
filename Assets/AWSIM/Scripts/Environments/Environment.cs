using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    using Geographic;

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
        string mgrsGridZone = "54SUE";

        [SerializeField]
        GeoCoordinate worldOriginGeoCoordinate = new GeoCoordinate(35.499810, 138.854828, 0);

        /// <summary>
        /// Reference point of MGRS coordinate system.
        /// </summary>
        public Vector3 MgrsOffsetPosition => mgrsOffsetPosition;

        /// <summary>
        /// MGRS Grid Zone. (e.g. Tokyo is "54SUE")
        /// </summary>
        /// <see href="https://maps.gsi.go.jp/#9/35.499810/138.854828/&base=std&ls=std&disp=1&vs=c1g1j0h0k0l0u1t0z0r0s0m0f1"></see>
        public string MgrsGridZone => mgrsGridZone;

        public GeoCoordinate WorldOriginGeoCoordinate => worldOriginGeoCoordinate;
    }
}
