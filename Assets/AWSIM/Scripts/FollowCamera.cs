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
        [Tooltip("Transform of object to follow")]
        public Transform target;

        [Tooltip("The distance between the camera and the target object")]
        public float Distance = 10.0f;

        [Tooltip("Lateral offset of the camera position")]
        public float Offset = 0.0f;

        [Tooltip("Height of the camera above the target object")]
        public float Height = 5.0f;

        [Tooltip("Camera height multiplier")]
        public float HeightMultiplier = 0.5f;

        [Tooltip("Maximum camera rotation speed around the target")]
        public float MaxRotateAroundSpeed = 32.0f;

        private float heightDamping = 2.0f;

        private float rotateAroundSpeed = 0.0f;
        private float currentCameraDirection = 0.0f;

        void Update()
        {
            // rotate around to left
            if(Input.GetKey(KeyCode.Keypad1))
            {
                rotateAroundSpeed = MaxRotateAroundSpeed;
            }
            // rotate around to right
            else if(Input.GetKey(KeyCode.Keypad3))
            {
                rotateAroundSpeed = MaxRotateAroundSpeed * -1f;
            }
            // back view
            else if(Input.GetKeyDown(KeyCode.Keypad2))
            {
                rotateAroundSpeed = 0f;
                currentCameraDirection = 0.0f;
            }
            // front view
            else if(Input.GetKeyDown(KeyCode.Keypad8))
            {
                rotateAroundSpeed = 0f;
                currentCameraDirection = 180.0f;
            }
            // left view
            else if(Input.GetKeyDown(KeyCode.Keypad4))
            {
                rotateAroundSpeed = 0f;
                currentCameraDirection = 90.0f;
            }
            // right view
            else if(Input.GetKeyDown(KeyCode.Keypad6))
            {
                rotateAroundSpeed = 0f;
                currentCameraDirection = 270.0f;
            }
            else
            {
                rotateAroundSpeed = 0f;
            }
        }

        void LateUpdate()
        {
#if UNITY_EDITOR
            if (Time.deltaTime == 0.0f)
                return;
#endif
            if (target == null)
                return;

            // calculate height for camera
            float newHeight = target.position.y + Height;
            float currentCameraHeight = transform.position.y;
            currentCameraHeight = Mathf.Lerp(currentCameraHeight, newHeight, heightDamping * Time.deltaTime);

            // calculate rotation for camera
            float currentRotationAngle = target.eulerAngles.y;
            Quaternion currentCameraRotation = Quaternion.Euler(0, currentRotationAngle, 0);

            // include additional rotation for camera rotating around target
            currentCameraDirection += rotateAroundSpeed * Time.deltaTime;
            if(currentCameraDirection > 360)
            {
                currentCameraDirection -= 360f;
            }
            if(currentCameraDirection < -360f)
            {
                currentCameraDirection += 360f;
            }
            currentCameraRotation *= Quaternion.Euler(0f, currentCameraDirection, 0f);

            // set camera position and orientation
            Vector3 pos = target.position;
            pos -= (currentCameraRotation * Vector3.forward * Distance + currentCameraRotation * Vector3.right * Offset);
            pos.y = currentCameraHeight;

            transform.position = pos;
            transform.LookAt(target.position + Vector3.up * Height * HeightMultiplier);
        }


    }
}
