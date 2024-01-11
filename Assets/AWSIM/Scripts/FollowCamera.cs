using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// Camera script that follows the target.
    /// </summary>
    public class FollowCamera : MonoBehaviour
    {
        public Transform target;
        public float Distance = 10.0f;
        public float Offset = 0.0f;
        public float Height = 5.0f;
        public float HeightMultiplier = 0.5f;
        float HeightDamping = 2.0f;
        float RotationDamping = 1.5f;


        float targetAngle = 0.0f;

        void LateUpdate()
        {
#if UNITY_EDITOR
            if (Time.deltaTime == 0.0f)
                return;
#endif
            if (target == null)
                return;

            targetAngle = target.eulerAngles.y;

            float newHeight = target.position.y + Height;
            float currentRotationAngle = target.eulerAngles.y;
            float currentHeight = transform.position.y;

            currentRotationAngle = Mathf.LerpAngle(currentRotationAngle, targetAngle, RotationDamping * Time.deltaTime);
            currentHeight = Mathf.Lerp(currentHeight, newHeight, HeightDamping * Time.deltaTime);
            Quaternion currentRotation = Quaternion.Euler(0, currentRotationAngle, 0);
            transform.position = target.position;
            transform.position -= (currentRotation * Vector3.forward * Distance + currentRotation * Vector3.right * Offset);
            Vector3 pos = transform.position;
            pos.y = currentHeight;
            transform.position = pos;
            transform.LookAt(target.position + Vector3.up * Height * HeightMultiplier);
        }
    }
}
