// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
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
            /// <summary>
            /// Specifies the initial velocity of a smoke particle in the x-z plane in [m/s].
            /// </summary>
            public float InitialPlaneVelocity => _initialPlaneVelocity;

            /// <summary>
            /// Specifies the vertical velocity of a smoke particle in [m/s].
            /// </summary>
            public float InitialVerticalVelocity => _initialVerticalVelocity;

            /// <summary>
            /// Specifies the acceleration of a smoke particle in the x-z plane in [m/s^2].
            /// </summary>
            public float PlaneAcceleration => _planeAcceleration;

            /// <summary>
            /// Specifies the vertical acceleration of a smoke particle in [m/s^2].
            /// </summary>
            public float VerticalAcceleration => _verticalAcceleration;

            [Tooltip("Specifies the initial velocity of a smoke particle in the x-z plane in [m/s].")]
            [SerializeField]
            [Range(0.0f, 0.5f)]
            float _initialPlaneVelocity;
            
            
            [Tooltip("Specifies the vertical velocity of a smoke particle in [m/s].")]
            [SerializeField]
            [Range(-0.5f, 3.0f)]
            float _initialVerticalVelocity;
            

            [Tooltip("Specifies the acceleration of a smoke particle in the x-z plane in [m/s^2].")]
            [SerializeField]
            [Range(-0.1f, 0.1f)]
            float _planeAcceleration;
            

            [Tooltip("Specifies the vertical acceleration of a smoke particle in [m/s^2].")]
            [SerializeField]
            [Range(-0.1f, 0.1f)]
            float _verticalAcceleration;
            
        }

        /// <summary>
        /// Specifies the maximum number of smoke particles in a scene.
        /// </summary>
        public int MaxParticle => _maxParticle;
        /// <summary>
        /// Specifies the radius of a circular region where particles are randomly generated in [m].
        /// </summary>
        public float ParticleRangeRadius => _particleRangeRadius;
        /// <summary>
        /// Specifies the size of a smoke particle in [m].
        /// </summary>
        public float ParticleSize => _particleSize;
        /// <summary>
        /// Specifies the average lifetime of a particle in [s].
        /// </summary>
        public float AverageLifetime => _averageLifetime;
        /// <summary>
        /// Specifies the variation range of lifetime of a particle in [s].
        /// </summary>
        public float VariationLifetime => _variationLifetime;


        [Tooltip("Specifies the maximum number of smoke particles in a scene.")]
        [SerializeField]
        [Range(1, 1000)]
        int _maxParticle = 250;

        [Tooltip("Specifies the radius of a circular region where particles are randomly generated in [m].")]
        [SerializeField]
        [Range(0.01f, 3.0f)]
        float _particleRangeRadius = 2.0f;
        

        [Tooltip("Specifies the size of a smoke particle in [m].")]
        [SerializeField]
        [Range(0.005f, 0.1f)]
        float _particleSize = 0.07f;
        

        [Tooltip("Specifies the average lifetime of a particle in [s].")]
        [SerializeField]
        [Range(5.0f, 15.0f)]
        float _averageLifetime = 7.5f;
        

        [Tooltip("Specifies the variation range of lifetime of a particle in [s].")]
        [SerializeField]
        [Range(0.0f, 5.0f)]
        float _variationLifetime = 2.5f;
        

        [SerializeField]
        SmokeParticlePhysics _physics;

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
        
        /// <summary>
        /// Returns initial velocity and acceleration of particle.
        /// </summary>
        public float[] GetVelAcc()
        {
            return new float[4] { _physics.InitialPlaneVelocity, _physics.InitialVerticalVelocity, _physics.PlaneAcceleration, _physics.VerticalAcceleration };
        }

        /// <summary>
        /// Returns lifetime of particle.
        /// </summary>
        public double GetLifetime()
        {
            return (double)(AverageLifetime + UnityEngine.Random.Range(-VariationLifetime, VariationLifetime));
        }

        void CreateSmokeParticle()
        {
            float angleRad = UnityEngine.Random.Range(0.0f, (float)System.Math.PI * 2.0f);
            float radius = UnityEngine.Random.Range(0.0f, ParticleRangeRadius);
            SmokeParticle.Create(gameObject, radius, angleRad);
        }
    }
}