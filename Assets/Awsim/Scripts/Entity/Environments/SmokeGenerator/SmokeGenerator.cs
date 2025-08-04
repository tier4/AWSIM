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
        [Tooltip("Specifies the maximum number of smoke particles in a scene.")]
        [SerializeField]
        [Range(1, 1000)]
        private int _maxParticle = 250;
        [Tooltip("Specifies the radius of a circular region where particles are randomly generated in [m].")]
        [SerializeField]
        [Range(0.01f, 3.0f)]
        private float _particleRangeRadius = 2.0f;
        [Tooltip("Specifies the size of a smoke particle in [m].")]
        [SerializeField]
        [Range(0.005f, 0.1f)]
        private float _particleSize = 0.07f;
        [Tooltip("Specifies the average lifetime of a particle in [s].")]
        [SerializeField]
        [Range(5.0f, 15.0f)]
        private float _averageLifetime = 7.5f;
        [Tooltip("Specifies the variation range of lifetime of a particle in [s].")]
        [SerializeField]
        [Range(0.0f, 5.0f)]
        private float _variationLifetime = 2.5f;

        [SerializeField]
        private SmokeParticlePhysics _physics;

        public void Initialize()
        {
            for (int i = 0; i < _maxParticle; i++)
            {
                this.CreateSmokeParticle();
            }
            Debug.Log("SmokeGenerator initialized with material: " + GetComponent<MeshRenderer>().material.name);
        }

        public void OnUpdate()
        {
            if (this.transform.childCount < _maxParticle)
            {
                this.CreateSmokeParticle();
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
            float angleRad = Random.Range(0.0f, (float)System.Math.PI * 2.0f);
            float radius = Random.Range(0.0f, _particleRangeRadius);
            SmokeParticle.Create(gameObject, _particleSize, radius, angleRad);
        }

        /// <summary>
        /// Returns particle size.
        /// </summary>
        public float GetParticleSize()
        {
            return this._particleSize;
        }

        /// <summary>
        /// Returns initial velocity and acceleration of particle.
        /// </summary>
        public float[] GetVelAcc()
        {
            return new float[4] {this._physics.initialPlaneVelocity, this._physics.initialVerticalVelocity, this._physics.planeAcceleration, this._physics.verticalAcceleration};
        }

        /// <summary>
        /// Returns lifetime of particle.
        /// </summary>
        public double GetLifetime()
        {
            return (double)(_averageLifetime + Random.Range(-_variationLifetime, _variationLifetime));
        }
    }
}