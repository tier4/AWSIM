using System;
using UnityEngine;

/// <summary>
/// Struct for storing Smoke Particle class-related parameters.
/// </summary>
[Serializable]
public struct SmokeParticlePhysics
{
    [Tooltip("Specifies the initial velocity of a smoke particle in the x-z plane in [m/s].")]
    [Range(0.0f, 0.5f)]
    public float initialPlaneVelocity;
    [Tooltip("Specifies the vertical velocity of a smoke particle in [m/s].")]
    [Range(-0.5f, 3.0f)]
    public float initialVerticalVelocity;

    [Tooltip("Specifies the acceleration of a smoke particle in the x-z plane in [m/s^2].")]
    [Range(-0.1f, 0.1f)]
    public float planeAcceleration;
    [Tooltip("Specifies the vertical acceleration of a smoke particle in [m/s^2].")]
    [Range(-0.1f, 0.1f)]
    public float verticalAcceleration;
}