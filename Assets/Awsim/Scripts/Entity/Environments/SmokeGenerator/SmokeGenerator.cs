using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Awsim.Entity
{ 
    [RequireComponent(typeof (MeshRenderer))]

    /// <summary>
    /// Smoke Generator class.
    /// </summary>
    [System.Serializable]
    public class SmokeGenerator : MonoBehaviour
    {
        /// <summary>
        /// Struct for storing Smoke Particle class-related parameters.
        /// </summary>
        [Serializable]
        public struct SmokeParticlePhysics
        {
            [Tooltip("Specifies the initial velocity of a smoke particle in the x-z plane in [m/s].")]
            [SerializeField]
            [Range(0.0f, 0.5f)]
            private float _initialPlaneVelocity;
            public float InitialPlaneVelocity => _initialPlaneVelocity;

            [Tooltip("Specifies the vertical velocity of a smoke particle in [m/s].")]
            [SerializeField]
            [Range(-0.5f, 3.0f)]
            private float _initialVerticalVelocity;
            public float InitialVerticalVelocity => _initialVerticalVelocity;

            [Tooltip("Specifies the acceleration of a smoke particle in the x-z plane in [m/s^2].")]
            [SerializeField]
            [Range(-0.1f, 0.1f)]
            private float _planeAcceleration;
            public float PlaneAcceleration => _planeAcceleration;
            
            [Tooltip("Specifies the vertical acceleration of a smoke particle in [m/s^2].")]
            [SerializeField]
            [Range(-0.1f, 0.1f)]
            private float _verticalAcceleration;
            public float VerticalAcceleration => _verticalAcceleration;
        }

        [Tooltip("Specifies the maximum number of smoke particles in a scene.")]
        [SerializeField]
        [Range(1, 1000)]
        private int _maxParticle = 250;
        public int MaxParticle => _maxParticle;

        [Tooltip("Specifies the radius of a circular region where particles are randomly generated in [m].")]
        [SerializeField]
        [Range(0.01f, 3.0f)]
        private float _particleRangeRadius = 2.0f;
        public float ParticleRangeRadius => _particleRangeRadius;

        [Tooltip("Specifies the size of a smoke particle in [m].")]
        [SerializeField]
        [Range(0.005f, 0.1f)]
        private float _particleSize = 0.07f;
        public float ParticleSize => _particleSize;

        [Tooltip("Specifies the average lifetime of a particle in [s].")]
        [SerializeField]
        [Range(5.0f, 15.0f)]
        private float _averageLifetime = 7.5f;
        public float AverageLifetime => _averageLifetime;

        [Tooltip("Specifies the variation range of lifetime of a particle in [s].")]
        [SerializeField]
        [Range(0.0f, 5.0f)]
        private float _variationLifetime = 2.5f;
        public float VariationLifetime => _variationLifetime;

        [SerializeField]
        private SmokeParticlePhysics _physics;

        public void Initialize()
        {
            for (int i = 0; i < MaxParticle; i++)
            {
                CreateSmokeParticle();
            }
            Debug.Log("SmokeGenerator initialized with material: " + GetComponent<MeshRenderer>().material.name);
        }

        public void OnUpdate()
        {
            if (transform.childCount < MaxParticle)
            {
                CreateSmokeParticle();
            }

            foreach (Transform child in transform)
            {
                var smoke = child.GetComponent<SmokeParticle>();
                if (smoke != null)
                {
                    smoke.OnUpdate();
                }
            }
        }
        
        private void CreateSmokeParticle()
        {
            float angleRad = UnityEngine.Random.Range(0.0f, (float)System.Math.PI * 2.0f);
            float radius = UnityEngine.Random.Range(0.0f, ParticleRangeRadius);
            SmokeParticle.Create(gameObject, radius, angleRad);
        }

        /// <summary>
        /// Returns initial velocity and acceleration of particle.
        /// </summary>
        public float[] GetVelAcc()
        {
            return new float[4] {_physics.InitialPlaneVelocity, _physics.InitialVerticalVelocity, _physics.PlaneAcceleration, _physics.VerticalAcceleration};
        }

        /// <summary>
        /// Returns lifetime of particle.
        /// </summary>
        public double GetLifetime()
        {
            return (double)(AverageLifetime + UnityEngine.Random.Range(-VariationLifetime, VariationLifetime));
        }
    }
}