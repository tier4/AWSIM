using UnityEngine;

[RequireComponent(typeof (MeshFilter))]
[RequireComponent(typeof (MeshRenderer))]

public class SmokeParticle : MonoBehaviour
{
    void Start()
    {
        this.CreateCube();
    }

    void Update()
    {
        this.transform.position += new Vector3(0.0f, 0.005f, 0.0f);
        if (this.transform.position.y >= 5.0)
            Destroy(gameObject);
        
        /* if (Input.GetKey(KeyCode.UpArrow))
            gameObject.transform.position += new Vector3(0.0f, 0.005f, 0.0f);
        if (Input.GetKey(KeyCode.DownArrow))
            gameObject.transform.position -= new Vector3(0.0f, 0.01f, 0.0f); */
        
    }

    private void CreateCube ()
    {
        float size = 0.01f;
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
			
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.Clear ();
		mesh.vertices = vertices;
		mesh.triangles = triangles;
		mesh.Optimize ();
		mesh.RecalculateNormals ();
	}

    public static GameObject Create(GameObject gameObject)
    {
        float radius = 0.5f;

        GameObject particle = new GameObject("Particle");
        particle.transform.parent = gameObject.transform;
        particle.transform.position = gameObject.transform.position + new Vector3(Random.Range(-radius, radius), Random.Range(0, radius), Random.Range(-radius, radius));
        particle.AddComponent(typeof(MeshFilter));
        particle.AddComponent(typeof(MeshRenderer));
        particle.AddComponent<SmokeParticle>();

        return particle;
    }
}