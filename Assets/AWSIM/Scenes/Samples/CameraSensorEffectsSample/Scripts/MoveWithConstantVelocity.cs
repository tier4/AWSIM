using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoveWithConstantVelocity : MonoBehaviour
{

    [SerializeField] private float speed = 0f;
    [SerializeField] private Vector3 direction = Vector3.zero;
    [SerializeField] private Rigidbody rigidbody = default; 

    void Start()
    {
        if(rigidbody != null)
        {
            rigidbody.velocity = direction.normalized * speed;
        }
    }
}
