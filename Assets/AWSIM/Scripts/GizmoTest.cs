using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Utils;
using AWSIM;

public class GizmoTest : MonoBehaviour
{
    /// Steering angle when the vehicle turns.
    public float SteeringAngle = 35f;

    void OnDrawGizmos()
    {
        /// Theoretical turning radious calculated based Bicycle Model.
        float ang = Mathf.Asin(1 / Mathf.Sqrt(Mathf.Pow((1 / Mathf.Tan(SteeringAngle * Mathf.Deg2Rad) + 1.8199022f / 2.787877f), 2) + 1));
        float rad = 2.787877f / Mathf.Sin(ang);
        Vector3 center = new Vector3 (rad, 0, 0);

        Gizmos.color = Color.blue;
        GizmosExtensions.DrawWireCircle (center, rad, 100);
    }
}