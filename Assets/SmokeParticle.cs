using UnityEngine;

[RequireComponent(typeof (MeshFilter))]
[RequireComponent(typeof (MeshRenderer))]

public class SmokeParticle : MonoBehaviour
{
	private SmokeGenerator parentComp;
	private double lifeTime = 7.5;
	private Vector3 velocity = new Vector3(0.0f, -0.05f, 0.0f);
	private Vector3 acceleration = new Vector3(0.0f, 0.05f, 0.0f);
	
    void Start()
    {
        this.CreateCube();
		this. lifeTime += (double)Random.Range(-2.5f, 2.5f);

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
        float height, width, depth;
        height = size; width = size; depth = size;

		Vector3[] vertices = {
			new Vector3 (0, 0, 0),
			new Vector3 (width, 0, 0),
			new Vector3 (width, height, 0),
			new Vector3 (0, height, 0),
			new Vector3 (0, height, depth),
			new Vector3 (width, height, depth),
			new Vector3 (width, 0, depth),
			new Vector3 (0, 0, depth),
		};

		int[] triangles = {
			0, 2, 1, //face front
			0, 3, 2,
			2, 3, 4, //face top
			2, 4, 5,
			1, 2, 5, //face right
			1, 5, 6,
			0, 7, 4, //face left
			0, 4, 3,
			5, 4, 7, //face back
			5, 7, 6,
			0, 6, 7, //face bottom
			0, 1, 6
		};
			
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		mesh.Clear ();
		mesh.vertices = vertices;
		mesh.triangles = triangles;
		mesh.Optimize ();
		mesh.RecalculateNormals ();
	}

	public void SetParentComp(GameObject gameObject)
	{
		this.parentComp = gameObject.GetComponentInParent<SmokeGenerator>();
	}

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
