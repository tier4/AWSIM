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

        [Space(10)]
        [Tooltip("Transform of object to follow")]
        public Transform target;

        [Header("Base Settings")]
        [Tooltip("Base distance between the camera and the target object")]
        public float Distance = 10.0f;

        [Tooltip("Lateral offset of the camera position")]
        public float Offset = 0.0f;

        [Tooltip("Base height of the camera above the target object")]
        public float Height = 5.0f;

        [Tooltip("Camera height multiplier")]
        public float HeightMultiplier = 0.5f;

        [Space(10)]
        [Header("Camera Movement Settings")]
        [Tooltip("Toggle key between rotate around mode and follow mode")]
        public bool EnableRotateAroundMode = false;
        private bool lastEnableRotateAroundMode = false;

        [Space(10)]
        [Tooltip("Mouse movement sensitivity for camera rotation around the target")]
        [Range(0.0f, 100.0f)]
        public float RotateAroundSensitivity = 16.0f;

        [Tooltip("Mouse movement sensitivity for camera height adjustment")]
        [Range(0.0f, 20.0f)]
        public float HeightAdjustmentSensitivity = 1.0f;

        [Tooltip("Mouse scroll wheel sensitivity for camera zoom")]
        [Range(50.0f, 300.0f)]
        public float ZoomSensitivity = 200f;

        [Space(10)]
        [Tooltip("Invert horizontal mouse movement")]
        public bool InvertHorzAxis = false;

        [Tooltip("Invert vertical mouse movement")]
        public bool InvertVertAxis = false;

        [Tooltip("Invert mouse scroll wheel")]
        public bool InvertScrollWheel = false;

        [Space(10)]
        [Tooltip("Maximum value of camera height")]
        public float MaxHeight = 10f;

        [Tooltip("Minimum value of camera distance to target object")]
        public float MinDistance = 1f;

        [Tooltip("Maximum value of camera distance to target object")]
        public float MaxDistance = 20f;

        #endregion

        #region [Private Vars]

        private float distanceAdjustmentSpeed = 0f;
        private float currentDistance = 10.0f;
        private float heightDamping = 2.0f;
        private float rotateAroundSpeed = 0.0f;
        private float currentCameraDirection = 0.0f;
        private bool rotateCameraAroundActive = false;

        private float currentHeight = 0f;
        private float heightAdjustmentSpeed = 0f;
        private float deltaHeight = 0f;

        #endregion

        #region [Init Vars]

        void Awake()
        {
            distanceAdjustmentSpeed = 0.0f;
            currentDistance = Distance;
            rotateAroundSpeed = 0.0f;
            currentCameraDirection = 0.0f;
            rotateCameraAroundActive = false;

            currentHeight = Height;
            heightAdjustmentSpeed = 0f;
            deltaHeight = 0f;
        }

        #endregion

        #region [Unity Event Methods]

        void Update()
        {
            // turn on or off the camera rotate around feature
            if (EnableRotateAroundMode != lastEnableRotateAroundMode)
            {
                lastEnableRotateAroundMode = EnableRotateAroundMode;

                rotateCameraAroundActive = !rotateCameraAroundActive;
                onActivateRotateCameraAround?.Invoke(rotateCameraAroundActive);
            }

            if (!rotateCameraAroundActive)
            {
                return;
            }

            if (Input.GetKey(KeyCode.LeftShift))
            {
                // front view
                if (Input.GetKey(KeyCode.UpArrow))
                {
                    rotateAroundSpeed = 0f;
                    heightAdjustmentSpeed = 0f;
                    currentHeight = transform.position.y - target.position.y;
                    distanceAdjustmentSpeed = 0f;
                    currentCameraDirection = 180f;
                }
                // back view
                else if (Input.GetKey(KeyCode.DownArrow))
                {
                    rotateAroundSpeed = 0f;
                    heightAdjustmentSpeed = 0f;
                    currentHeight = transform.position.y - target.position.y;
                    distanceAdjustmentSpeed = 0f;
                    currentCameraDirection = 0f;
                }
                // left view
                else if (Input.GetKey(KeyCode.LeftArrow))
                {
                    rotateAroundSpeed = 0f;
                    heightAdjustmentSpeed = 0f;
                    currentHeight = transform.position.y - target.position.y;
                    distanceAdjustmentSpeed = 0f;
                    currentCameraDirection = 90f;
                }
                // right view
                else if (Input.GetKey(KeyCode.RightArrow))
                {
                    rotateAroundSpeed = 0f;
                    heightAdjustmentSpeed = 0f;
                    currentHeight = transform.position.y - target.position.y;
                    distanceAdjustmentSpeed = 0f;
                    currentCameraDirection = 270f;
                }

                // get horizontal mouse movement for camera rotation
                float mouseHorzAxis = Input.GetAxis("Mouse X");
                if (Mathf.Abs(mouseHorzAxis) > 0.01f)
                {
                    rotateAroundSpeed = RotateAroundSensitivity * mouseHorzAxis * currentDistance * (InvertHorzAxis ? 1f : -1f);
                }
                else
                {
                    rotateAroundSpeed = 0f;
                }

                // get vertical mouse movement for camera height adjustment
                float mouseVertAxis = Input.GetAxis("Mouse Y");
                if (Mathf.Abs(mouseVertAxis) > 0.01f)
                {
                    heightAdjustmentSpeed = HeightAdjustmentSensitivity * mouseVertAxis * currentDistance * (InvertVertAxis ? -1f : 1f);
                }
                else
                {
                    heightAdjustmentSpeed = 0f;
                }

                // get mouse scroll whell for camera zoom
                float mouseScroll = Input.GetAxis("Mouse ScrollWheel");
                if (Mathf.Abs(mouseScroll) > 0.01f)
                {
                    distanceAdjustmentSpeed = ZoomSensitivity * mouseScroll * (InvertScrollWheel ? 1f : -1f);
                }
                else
                {
                    distanceAdjustmentSpeed = 0f;
                }

            }
            else
            {
                rotateAroundSpeed = 0f;
                heightAdjustmentSpeed = 0f;
                distanceAdjustmentSpeed = 0f;
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

            // calculate additional rotation, height and distance for camera
            if(rotateCameraAroundActive)
            {
                // include additional rotation for camera rotating around target
                currentCameraDirection += rotateAroundSpeed * Time.deltaTime / Time.timeScale;
                if(currentCameraDirection > 360)
                {
                    currentCameraDirection -= 360f;
                }
                if(currentCameraDirection < -360f)
                {
                    currentCameraDirection += 360f;
                }

                // include additional height for camera above target
                deltaHeight = heightAdjustmentSpeed * Time.deltaTime / Time.timeScale;
                if(currentHeight + deltaHeight > MaxHeight)
                {
                    deltaHeight = MaxHeight - currentHeight;
                    currentHeight = MaxHeight;
                }
                else if(currentHeight + deltaHeight < Height)
                {
                    deltaHeight = Height - currentHeight;
                    currentHeight = Height;
                }
                else
                {
                    currentHeight += deltaHeight;
                }

                // include additional distance between camera and target
                currentDistance += distanceAdjustmentSpeed * Time.deltaTime / Time.timeScale;
                currentDistance = Mathf.Clamp(currentDistance, MinDistance, MaxDistance);
            }
            // set camera position to base values
            else
            {
                currentCameraDirection = 0f;
                currentDistance = Distance;
                currentHeight = Height;
                deltaHeight = 0f;
            }

            float newHeight = target.position.y + currentHeight;
            float currentCameraHeight = transform.position.y + deltaHeight;
            currentCameraHeight = Mathf.Lerp(currentCameraHeight, newHeight, heightDamping * Time.deltaTime);

            // calculate rotation for camera
            float currentRotationAngle = target.eulerAngles.y;
            Quaternion currentCameraRotation = Quaternion.Euler(0, currentRotationAngle, 0);
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
