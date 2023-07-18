using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Utils;
using AWSIM;

public class GizmoTest : MonoBehaviour
{
    GameObject Lexus;
    Vehicle vehicle;

    void Start()
    {
        Lexus = GameObject.Find("Lexus RX450h 2015 Sample Sensor");
        vehicle = Lexus.GetComponent<Vehicle>();
    }

    void OnDrawGizmos()
    {
        float SteeringAngle = 35f;

        float ang1 = SteeringAngle * Mathf.Deg2Rad;
        float ang2 = Mathf.Asin(1 / Mathf.Sqrt(Mathf.Pow((1 / Mathf.Tan(ang1) + 1.8199022f / 2.787877f), 2) + 1));

        float rad = 2.787877f / Mathf.Sin(ang2);

        Gizmos.color = Color.blue;
        GizmosExtensions.DrawWireCircle (new Vector3 (rad, 0, 0), rad, 100);

        Debug.Log(vehicle.LocalVelocity);
    }
}
