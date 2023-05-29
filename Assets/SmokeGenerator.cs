using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SmokeGenerator : MonoBehaviour
{
    int max_particle = 100;

    // Start is called before the first frame update
    void Start()
    {
        for (int i = 0; i < max_particle; i++)
        {
            SmokeParticle.Create(gameObject);
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
