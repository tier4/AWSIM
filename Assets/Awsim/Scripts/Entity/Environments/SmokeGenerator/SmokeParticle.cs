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

using UnityEngine;

namespace Awsim.Entity
{ 
	[RequireComponent(typeof(MeshFilter))]
	[RequireComponent(typeof (MeshRenderer))]

	/// <summary>
	/// Smoke Particle class.
	/// </summary>
	public class SmokeParticle : MonoBehaviour
	{
		SmokeGenerator _parentComp;
		double _lifeTime;
		Vector3 _velocity = new Vector3(0.0f, -0.05f, 0.0f);
		Vector3 _acceleration = new Vector3(0.0f, 0.05f, 0.0f);
		
        public void Initialize()
		{
		    CreateCube();
		    _lifeTime = _parentComp.GetLifetime();

		    Material mat = _parentComp.GetComponent<MeshRenderer>().material;
		    MeshRenderer rend = GetComponent<MeshRenderer>();
		    if (rend != null)
		        rend.material = mat;
		}

		public void OnUpdate()
		{
		    _velocity += _acceleration * Time.deltaTime;
			Vector3 displacement = _velocity * Time.deltaTime;
		    transform.position += displacement;

		    _lifeTime -= Time.deltaTime;
		    if (_lifeTime <= 0.0)
		        Destroy(gameObject);
		}

		/// <summary>
		/// Creates a new Smoke Particle GameObject of specified properties.
		/// </summary>
		/// <param name="gameObject">Parent GameObject of the Smoke Particle.</param>
		/// <param name="radius">Radius of a circle which defines the region where the Smoke Particle is generated in.</param>
		/// <param name="angleRad">Angular location of the Smoke Particle to be generated at.</param>
		public static void Create(GameObject gameObject, float radius, float angleRad)
		{
			GameObject particle = new GameObject("Particle");
			particle.transform.parent = gameObject.transform;

			float x = radius * (float)System.Math.Cos(angleRad);
			float z = radius * (float)System.Math.Sin(angleRad);
			float y = Random.Range(0.0f, 1.5f);
			particle.transform.position = gameObject.transform.position + new Vector3(x, y, z);

			particle.AddComponent(typeof(MeshFilter));
			particle.AddComponent(typeof(MeshRenderer));

			var smoke = particle.AddComponent<SmokeParticle>();
			smoke.SetParentComp(gameObject);
			smoke.SetVelAcc(angleRad);
			smoke.Initialize();
		}
		
		void CreateCube()
		{
			float size = _parentComp.ParticleSize;

			Vector3[] vertices = {
				new Vector3 (0, 0, 0),
				new Vector3 (size, 0, 0),
				new Vector3 (size, size, 0),
				new Vector3 (0, size, 0),
				new Vector3 (0, size, size),
				new Vector3 (size, size, size),
				new Vector3 (size, 0, size),
				new Vector3 (0, 0, size),
			};

			int[] triangles = {
				0, 2, 1,
				0, 3, 2,
				2, 3, 4,
				2, 4, 5,
				1, 2, 5,
				1, 5, 6,
				0, 7, 4,
				0, 4, 3,
				5, 4, 7,
				5, 7, 6,
				0, 6, 7,
				0, 1, 6
			};

			Mesh mesh = GetComponent<MeshFilter>().mesh;
			mesh.Clear();
			mesh.vertices = vertices;
			mesh.triangles = triangles;
			mesh.Optimize();
			mesh.RecalculateNormals();
		}

		/// <summary>
		/// Gets and sets the SmokeGenerator component of the parent GameObject.
		/// </summary>
		/// <param name="gameObject">Parent GameObject of the Smoke Particle.</param>
		void SetParentComp(GameObject gameObject)
		{
			_parentComp = gameObject.GetComponentInParent<SmokeGenerator>();
		}

		/// <summary>
		/// Sets the initial velocity and acceleration of Smoke Particle.
		/// </summary>
		/// <param name="angleRad">Angular location of Smoke Particle in radians.</param>
		void SetVelAcc(float angleRad)
		{
			float[] velAcc = _parentComp.GetVelAcc();
			float velPlane = velAcc[0];
			float velY = velAcc[1];
			float accPlane = velAcc[2];
			float accY = velAcc[3];

			float velX = velPlane * (float)System.Math.Cos(angleRad);
			float velZ = velPlane * (float)System.Math.Sin(angleRad);
			_velocity = new Vector3(velX, velY, velZ);

			float accX = accPlane * (float)System.Math.Cos(angleRad);
			float accZ = accPlane * (float)System.Math.Sin(angleRad);
			_acceleration = new Vector3(accX, accY, accZ);
		}
	}
}