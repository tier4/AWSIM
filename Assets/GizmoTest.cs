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
        //float v = Mathf.Sqrt(Mathf.Pow(vehicle.LocalVelocity.x, 2) + Mathf.Pow(vehicle.LocalVelocity.z, 2));
        //float v = 10f;
        float ang = SteeringAngle * Mathf.PI / 180f;
        //float cosBeta = vehicle.LocalVelocity.z / v;
        //float sinBeta = vehicle.LocalVelocity.x / v;

        //float rad = 2.5f * Mathf.Cos(Mathf.Acos(cosBeta) - vehicle.AngularVelocity.y * 1.367795f / v) / Mathf.Sin(2.5f * vehicle.AngularVelocity.y / v);
        //float rad = 1500f * Mathf.Pow(v, 2) / (2f * 2.5f * (0.1494f * 6000) * 0.5f / 1.367795f);

        float rad = 2.787877f / Mathf.Sin(ang);

        Gizmos.color = Color.blue;
        GizmosExtensions.DrawWireCircle (new Vector3 (rad, 0, 0), rad, 100);

        //Debug.Log(Mathf.Sin(ang) * Mathf.Cos(vehicle.AngularVelocity.y - ang) - Mathf.Cos(ang) * Mathf.Sin(vehicle.AngularVelocity.y - ang));
        Debug.Log(vehicle.LocalVelocity);
    }
}
