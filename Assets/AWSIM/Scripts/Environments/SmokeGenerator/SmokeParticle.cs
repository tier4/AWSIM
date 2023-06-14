using UnityEngine;

[RequireComponent(typeof (MeshFilter))]
[RequireComponent(typeof (MeshRenderer))]

/// <summary>
/// Smoke Particle class.
/// </summary>
public class SmokeParticle : MonoBehaviour
{
	private SmokeGenerator parentComp;
	private double lifeTime;
	private Vector3 velocity = new Vector3(0.0f, -0.05f, 0.0f);
	private Vector3 acceleration = new Vector3(0.0f, 0.05f, 0.0f);
	
    void Start()
    {
        this.CreateCube();
		this.lifeTime = parentComp.GetLifetime();

		Material mat = this.parentComp.GetComponent<MeshRenderer>().material;
		MeshRenderer rend = GetComponent<MeshRenderer>();
		if (rend != null)
			rend.material = mat;
    }

    void Update()
    {
		this.velocity += this.acceleration * Time.deltaTime;
		Vector3 displacement = this.velocity * Time.deltaTime;
		this.transform.position += displacement;

		this.lifeTime -= Time.deltaTime;
		if (lifeTime <= 0.0)
			Destroy(gameObject);
    }

    private void CreateCube ()
    {
		float size = this.parentComp.GetParticleSize();

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
		mesh.Clear ();
		mesh.vertices = vertices;
		mesh.triangles = triangles;
		mesh.Optimize ();
		mesh.RecalculateNormals ();
	}

	/// <summary>
	/// Gets and sets the SmokeGenerator component of the parent GameObject.
	/// </summary>
	/// <param name="gameObject">Parent GameObject of the Smoke Particle.</param>
	public void SetParentComp(GameObject gameObject)
	{
		this.parentComp = gameObject.GetComponentInParent<SmokeGenerator>();
	}

	/// <summary>
	/// Sets the initial velocity and acceleration of Smoke Particle.
	/// </summary>
	/// <param name="angleRad">Angular location of Smoke Particle in radians.</param>
	public void SetVelAcc(float angleRad)
	{
		float[] velAcc = this.parentComp.GetVelAcc();
		float velPlane = velAcc[0];
		float velY = velAcc[1];
		float accPlane = velAcc[2];
		float accY = velAcc[3];

		float velX = velPlane * (float)System.Math.Cos(angleRad);
		float velZ = velPlane * (float)System.Math.Sin(angleRad);
		this.velocity = new Vector3(velX, velY, velZ);

		float accX = accPlane * (float)System.Math.Cos(angleRad);
		float accZ = accPlane * (float)System.Math.Sin(angleRad);
		this.acceleration = new Vector3(accX, accY, accZ);
	}

	/// <summary>
	/// Creates a new Smoke Particle GameObject of specified properties.
	/// </summary>
	/// <param name="gameObject">Parent GameObject of the Smoke Particle.</param>
	/// <param name="particle_size">Edge length of the Smoke Particle in [m].</param>
	/// <param name="radius">Radius of a circle which defines the region where the Smoke Particle is generated in.</param>
	/// <param name="angleRad">Angular location of the Smoke Particle to be generated at.</param>
    public static void Create(GameObject gameObject, float particle_size, float radius, float angleRad)
    {
        GameObject particle = new GameObject("Particle");
        particle.transform.parent = gameObject.transform;

        float x = radius * (float)System.Math.Cos(angleRad);
        float z = radius * (float)System.Math.Sin(angleRad);
        float y = Random.Range(0.0f, 1.5f);
        particle.transform.position = gameObject.transform.position + new Vector3(x, y, z);

        particle.AddComponent(typeof(MeshFilter));
        particle.AddComponent(typeof(MeshRenderer));
		
        particle.AddComponent<SmokeParticle>();
		particle.GetComponent<SmokeParticle>().SetParentComp(gameObject);
		particle.GetComponent<SmokeParticle>().SetVelAcc(angleRad);
    }
}
