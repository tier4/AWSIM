using UnityEngine;

[RequireComponent(typeof (MeshFilter))]
[RequireComponent(typeof (MeshRenderer))]

public class SmokeParticle : MonoBehaviour
{
	float speed = 0.5f;
	double maxHeight = 2.5;

    void Start()
    {
        this.CreateCube();
    }

    void Update()
    {
		float disp = speed * (float)Time.deltaTime;
        this.transform.position += new Vector3(0.0f, disp, 0.0f);
        if (this.transform.position.y >= maxHeight)
            Destroy(gameObject);
    }

    private void CreateCube ()
    {
		var parentComp = gameObject.GetComponentInParent<SmokeGenerator>();
		float size = parentComp.GetParticleSize();
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

    public static void Create(GameObject gameObject, float particle_size, Vector3 particlePos)
    {
        GameObject particle = new GameObject("Particle");
        particle.transform.parent = gameObject.transform;
        particle.transform.position = particlePos;
        particle.AddComponent(typeof(MeshFilter));
        particle.AddComponent(typeof(MeshRenderer));
        particle.AddComponent<SmokeParticle>();
    }
}
