using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Utils;
using AWSIM;

public class GizmosTest : MonoBehaviour
{
    void OnDrawGizmos()
    {
        GameObject Lexus = GameObject.Find("Lexus RX450h 2015 Sample Sensor");
        Vehicle scripts =  Lexus.GetComponent<Vehicle>();


        /// Theoretical turning radious calculated based Bicycle Model.
        float ang = Mathf.Asin(1 / Mathf.Sqrt(Mathf.Pow((1 / Mathf.Tan(scripts.SteerAngle * Mathf.Deg2Rad) + scripts.tread / scripts.wheelBase), 2) + 1));
        float rad = 2.787877f / Mathf.Sin(ang);
        Vector3 center = new Vector3 (rad * Mathf.Cos(ang), 0.4438f, -rad * Mathf.Sin(ang));

        Gizmos.color = Color.blue;
        GizmosExtensions.DrawWireCircle (center, rad, 100);
    }
}