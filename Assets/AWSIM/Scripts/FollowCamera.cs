using System;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// Camera script that follows the target.
    /// </summary>
    public class FollowCamera : MonoBehaviour
    {
        #region [Events]

        /// <summary>
        /// Event dispatched when the Rotate Camera Around activation status changed.
        /// </summary>
        public event Action<bool> onActivateRotateCameraAround;

        #endregion

        #region [Public Vars]

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

        [Space(10)]
        [Header("Camera Zoom")]
        [Tooltip("Sensitivity of camera zoom")]
        public float ZoomSensitivity = 10f;

        [Space(10)]
        [Header("Camera Rotate Around")]
        [Tooltip("Toggle key between rotate around mode and follow mode")]
        public KeyCode RotateAroundModeToggle = KeyCode.C;

        [Tooltip("Maximum camera rotation speed around the target")]
        public float MaxRotateAroundSpeed = 64.0f;


        #endregion

        #region [Private Vars]

        private float currentDistance = 10.0f;
        private float heightDamping = 2.0f;
        private float rotateAroundSpeed = 0.0f;
        private float currentCameraDirection = 0.0f;
        private bool rotateCameraAroundActive = false;

        #endregion

        #region [Init Vars]

        void Awake()
        {
            currentDistance = Distance;
            rotateAroundSpeed = 0.0f;
            currentCameraDirection = 0.0f;
            rotateCameraAroundActive = false;
        }

        #endregion

        #region [Unity Event Methods]

        void Update()
        {
            // turn on or off the camera rotate around feature
            if(Input.GetKeyDown(RotateAroundModeToggle))
            {
                rotateCameraAroundActive = !rotateCameraAroundActive;
                onActivateRotateCameraAround?.Invoke(rotateCameraAroundActive);
            }

            if(!rotateCameraAroundActive)
            {
                return;
            }

            // rotate around when mouse middle button is held down
            if (Input.GetMouseButton(2))
            {
                if (Input.GetAxis("Mouse X") < 0)
                {
                    rotateAroundSpeed = MaxRotateAroundSpeed;
                }
                else if (Input.GetAxis("Mouse X") > 0)
                {
                    rotateAroundSpeed = MaxRotateAroundSpeed * -1f;
                }
                else
                {
                    rotateAroundSpeed = 0f;
                }
            }
            else
            {
                rotateAroundSpeed = 0f;
            }

            if(Input.GetAxis("Mouse ScrollWheel") < 0)
            {
                currentDistance += ZoomSensitivity * Time.deltaTime;
            }
            else if(Input.GetAxis("Mouse ScrollWheel") > 0)
            {
                currentDistance -= ZoomSensitivity * Time.deltaTime;
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
            if(rotateCameraAroundActive)
            {
                currentCameraDirection += rotateAroundSpeed * Time.deltaTime;
                if(currentCameraDirection > 360)
                {
                    currentCameraDirection -= 360f;
                }
                if(currentCameraDirection < -360f)
                {
                    currentCameraDirection += 360f;
                }
            }
            // set camera position to base values
            else
            {
                currentCameraDirection = 0f;
                currentDistance = Distance;
            }

            currentCameraRotation *= Quaternion.Euler(0f, currentCameraDirection, 0f);

            // set camera position and orientation
            Vector3 pos = target.position;
            pos -= (currentCameraRotation * Vector3.forward * currentDistance + currentCameraRotation * Vector3.right * Offset);
            pos.y = currentCameraHeight;

            transform.position = pos;
            transform.LookAt(target.position + Vector3.up * Height * HeightMultiplier);
        }

        #endregion

        #region [GUI]

        void OnGUI()
        {
            if(rotateCameraAroundActive)
            {
                GUIStyle style = new GUIStyle()
                {
                    fontSize = 18,
                    normal = new GUIStyleState()
                    {
                        textColor = Color.white,
                    },
                    alignment = TextAnchor.MiddleCenter,
                };
                GUI.Label(new Rect(Screen.width * 0.5f - 100f, 0f, 200f, 40f), "Camera Rotate Around Mode", style);
            }
        }

        #endregion
    }
}
