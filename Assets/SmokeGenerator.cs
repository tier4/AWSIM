using System.Collections;
using System.Collections.Generic;
using UnityEngine;


[RequireComponent(typeof (MeshRenderer))]

[System.Serializable]
public class SmokeGenerator : MonoBehaviour
{
    [SerializeField]
    [Range(0.005f, 0.1f)]
    private float particleSize = 0.05f;
    [SerializeField]
    [Range(1, 1000)]
    private int maxParticle = 100;
    [SerializeField]
    [Range(0.05f, 3.0f)]
    private float particleRangeRadius = 0.5f;

    // Start is called before the first frame update
    public void Start()
    {
        for (int i = 0; i < maxParticle; i++)
            this.CreateSmokeParticle();
        
        print(GetComponent<MeshRenderer>().material);
    }

    // Update is called once per frame
    public void Update()
    {
        if (this.transform.childCount < maxParticle)
            this.CreateSmokeParticle();
    }

    private void CreateSmokeParticle()
    {
        float angleRad = Random.Range(0.0f, (float)System.Math.PI*2.0f);
        float radius = Random.Range(0.0f, particleRangeRadius);
        float x = radius * (float)System.Math.Cos(angleRad);
        float z = radius * (float)System.Math.Sin(angleRad);
        float y = Random.Range(0.0f, 4.5f);
        Vector3 pos = this.transform.position + new Vector3(x, y, z);
        SmokeParticle.Create(gameObject, particleSize, pos);
    }

    public float GetParticleSize()
    {
        return this.particleSize;
    }
}
