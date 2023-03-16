using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace AWSIM
{
    /// <summary>
    /// NPC Vehicle class.
    /// Controlled by Position and Rotation.
    /// </summary>
    public class NPCVehicle : MonoBehaviour
    {
        public enum TurnSignalState
        {
            OFF,
            LEFT,
            RIGHT,
            HAZARD,
        }

        [Serializable]
        public class Wheel
        {
            public WheelCollider WheelCollider;
            public Transform VisualTransform;

            float wheelPitchAngle = 0;
            float lastSteerAngle = 0;

            public void UpdateVisual(float speed, float steerAngle)
            {
                // Apply WheelCollider position to visual object.
                WheelCollider.GetWorldPose(out var pos, out _);
                VisualTransform.position = pos;

                // wheel forward rotation(pitch).
                var additionalPitchAngle = (speed * Time.deltaTime / WheelCollider.radius) * Mathf.Rad2Deg;
                wheelPitchAngle += additionalPitchAngle;
                wheelPitchAngle %= 360;

                // steer angle.
                var fixedSteerAngle = Mathf.MoveTowardsAngle(lastSteerAngle, steerAngle, Time.deltaTime * maxSteerSpeed);

                // Apply rotations to visual wheel object.
                VisualTransform.localEulerAngles = new Vector3(wheelPitchAngle, fixedSteerAngle, 0);

                // Cache steer angle value for next update.
                lastSteerAngle = fixedSteerAngle;
            }
        }

        [Serializable]
        public class Axle
        {
            public Wheel leftWheel;
            public Wheel rightWheel;

            public void UpdateVisual(float speed, float steerAngle)
            {
                leftWheel.UpdateVisual(speed, steerAngle);
                rightWheel.UpdateVisual(speed, steerAngle);
            }
        }

        [Serializable]
        public class AxleSettings
        {
            [SerializeField] Axle frontAxle;
            [SerializeField] Axle rearAxle;

            public float GetWheelBase()
            {
                var frontPos = frontAxle.leftWheel.WheelCollider.transform.localPosition;
                var rearPos = rearAxle.leftWheel.WheelCollider.transform.localPosition;

                return Mathf.Abs(frontPos.z - rearPos.z);
            }

            public void UpdateVisual(float speed, float steerAngle)
            {
                frontAxle.UpdateVisual(speed, steerAngle);
                rearAxle.UpdateVisual(speed, 0);
            }
        }

        [SerializeField]
        GameObject visualObjectRoot;

        [Serializable]
        public class EmissionMaterial
        {
            [SerializeField] MeshRenderer meshRenderer;
            [SerializeField] int materialIndex;
            [SerializeField] float lightingIntensity;
            [SerializeField] Color lightingColor;
            [SerializeField, Range(0, 1)] float lightingExposureWeight;

            Material material = null;
            Color defaultEmissiveColor;
            float defaultExposureWeight;
            bool isOn = false;

            const string EmissiveColor = "_EmissiveColor";
            const string EmissiveExposureWeight = "_EmissiveExposureWeight";

            public void Initialize()
            {
                if (material == null)
                {
                    material = meshRenderer.materials[materialIndex];
                    material.EnableKeyword("_EMISSION");
                    defaultEmissiveColor = material.GetColor(EmissiveColor);
                    defaultExposureWeight = material.GetFloat(EmissiveExposureWeight);
                }
            }

            public void Set(bool isLightOn)
            {
                if (this.isOn == isLightOn)
                    return;

                this.isOn = isLightOn;
                if (isLightOn)
                {
                    material.SetColor(EmissiveColor, lightingColor * lightingIntensity);
                    material.SetFloat(EmissiveExposureWeight, lightingExposureWeight);
                }
                else
                {
                    material.SetColor(EmissiveColor, defaultEmissiveColor);
                    material.SetFloat(EmissiveExposureWeight, defaultExposureWeight);
                }
            }

            public void Destroy()
            {
                if (material != null)
                    UnityEngine.Object.Destroy(material);
            }
        }

        /// <summary>
        /// Current visualObject's activeself
        /// </summary>
        public bool VisualObjectRootActiveSelf => visualObjectRoot.activeSelf;

        /// <summary>
        /// Vehicle bounding box.
        /// </summary>
        public Bounds Bounds => bounds;

        /// <summary>
        /// Vehicle ID.
        /// </summary>
        public uint VehicleID { get; set; }

        // dynamics settings const values.
        const float maxSteerAngle = 40f;                    // deg
        const float maxSteerSpeed = 60f;                    // deg/s
        const float maxVerticalSpeed = 40;                  // m/s
        const float maxSlope = 45;                          // deg

        // light visual settings const values.
        const float turnSignalBlinkSec = 0.5f;             // seconds
        const float brakeLightAccelThreshold = -0.1f;      // m/s

        [Header("Physics Settings")]
        [SerializeField] Transform centerOfMass;
        [SerializeField] new Rigidbody rigidbody;
        [SerializeField] AxleSettings axleSettings;

        [Header("Bounding box Settngs")]
        [SerializeField] Bounds bounds;

        [Header("Brake light parameters")]
        [SerializeField] EmissionMaterial brakeLight;

        [Header("Turn signal parameters")]
        [SerializeField] EmissionMaterial leftTurnSignalLight;
        [SerializeField] EmissionMaterial rightTurnSignalLight;

        TurnSignalState turnSignalState = TurnSignalState.OFF;
        float turnSignalTimer = 0;
        bool currentTurnSignalOn = false;

        float wheelbase;        // m
        float acceleration;     // m/s^2
        Vector3 velocity;       // m/s
        float speed;            // m/s (forward only)
        float yawAngularSpeed;  // deg/s (yaw only)

        Vector3 lastVelocity;
        Vector3 lastPosition;
        float lastEulerAnguleY;
        float lastSpeed;

        // Start is called before the first frame update
        void Awake()
        {
            leftTurnSignalLight.Initialize();
            rightTurnSignalLight.Initialize();
            brakeLight.Initialize();

            rigidbody.centerOfMass = transform.InverseTransformPoint(centerOfMass.position);
            lastPosition = rigidbody.position;
            wheelbase = axleSettings.GetWheelBase();
        }

        // Update is called once per frame
        void Update()
        {
            // Update Wheel visuals.
            var steerAngle = CalcSteerAngle(speed, yawAngularSpeed, wheelbase);
            axleSettings.UpdateVisual(speed, steerAngle);

            // brake light.
            var isBrakeLightOn = IsBrakeLightOn();
            brakeLight.Set(isBrakeLightOn);

            // turn signal.
            if (IsAnyTurnSignalInputs() == false)
            {
                if (turnSignalTimer != 0)
                    turnSignalTimer = 0;

                if (currentTurnSignalOn != false)
                    currentTurnSignalOn = false;

                leftTurnSignalLight.Set(false);
                rightTurnSignalLight.Set(false);

                return;
            }

            turnSignalTimer -= Time.deltaTime;
            if (turnSignalTimer < 0f)
            {
                turnSignalTimer = turnSignalBlinkSec;
                currentTurnSignalOn = !currentTurnSignalOn;
            }

            var isLeftTurnSignalOn = IsLeftTurnSignalOn();
            leftTurnSignalLight.Set(isLeftTurnSignalOn);

            var isRightTurnSignalOn = IsRightTurniSignalOn();
            rightTurnSignalLight.Set(isRightTurnSignalOn);


            // --- inner methods ---
            static float CalcSteerAngle(float speed, float yawAngularSpeed, float wheelBase)
            {
                yawAngularSpeed *= Mathf.Deg2Rad;

                if (Mathf.Abs(yawAngularSpeed) < 0.01f || Mathf.Abs(speed) < 0.01f)
                {
                    return 0f;
                }
                var gyrationRadius = speed / Mathf.Tan(yawAngularSpeed);
                var yaw = Mathf.Asin(Mathf.Clamp(wheelBase / gyrationRadius, -1f, 1f)) * Mathf.Rad2Deg;
                yaw = Mathf.Clamp(yaw, -maxSteerAngle, maxSteerAngle);

                return yaw;
            }

            bool IsBrakeLightOn()
            {
                var isOn = false;
                if (speed < 0.03f)
                    isOn = true;
                //if (acceleration < brakeLightAccelThreshold)
                //    isOn = true;

                return isOn;
            }

            bool IsAnyTurnSignalInputs()
            {
                return turnSignalState == TurnSignalState.LEFT
                    || turnSignalState == TurnSignalState.RIGHT
                    || turnSignalState == TurnSignalState.HAZARD;
            }

            bool IsLeftTurnSignalOn()
            {
                return (turnSignalState == TurnSignalState.LEFT
                    || turnSignalState == TurnSignalState.HAZARD)
                    && currentTurnSignalOn;
            }

            bool IsRightTurniSignalOn()
            {
                return (turnSignalState == TurnSignalState.RIGHT
                    || turnSignalState == TurnSignalState.HAZARD)
                    && currentTurnSignalOn;
            }
        }

        void FixedUpdate()
        {
            // Calculate physical states for visual update.
            // velocity & speed.
            velocity = (rigidbody.position - lastPosition) / Time.deltaTime;
            speed = Vector3.Dot(velocity, transform.forward);

            // accleration.
            acceleration = (speed - lastSpeed) / Time.deltaTime;

            // yaw angular speed.
            yawAngularSpeed = (rigidbody.rotation.eulerAngles.y - lastEulerAnguleY) / Time.deltaTime;

            // TODO: set WheelCollider steer angle?

            // Cache current frame values.
            lastPosition = rigidbody.position;
            lastVelocity = velocity;
            lastEulerAnguleY = rigidbody.rotation.eulerAngles.y;
            lastSpeed = speed;
        }

        void Reset()
        {
            if (rigidbody == null)
                rigidbody = GetComponent<Rigidbody>();
        }

        void OnValidate()
        {
            rigidbody.isKinematic = false;
            rigidbody.useGravity = true;
            rigidbody.interpolation = RigidbodyInterpolation.Interpolate;
        }

        void OnDestroy()
        {
            leftTurnSignalLight.Destroy();
            rightTurnSignalLight.Destroy();
            brakeLight.Destroy();
        }

        /// <summary>
        /// Move the vehicle so that its x,z coordinates are the same as <paramref name="position"/>.<br/>
        /// Vertical movement is determined by physical operations that take effects of suspension and gravity into account.<br/>
        /// This method should be called from FixedUpdate because <see cref="Rigidbody"/> is updated internally.
        /// </summary>
        /// <param name="position">New position of the vehicle.</param>
        public void SetPosition(Vector3 position)
        {
            rigidbody.MovePosition(new Vector3(position.x, rigidbody.position.y, position.z));
            var velocityY = Mathf.Min(rigidbody.velocity.y, maxVerticalSpeed);
            rigidbody.velocity = new Vector3(0, velocityY, 0);
        }

        /// <summary>
        /// Rotate the vehicle so that its yaw is equal to that of <paramref name="rotation"/>.<br/>
        /// Pitch and roll are determined by physical operations that take effects of suspension and gravity into account.<br/>
        /// This method should be called from FixedUpdate because <see cref="Rigidbody"/> is updated internally.
        /// </summary>
        /// <param name="rotation">New rotation of the vehicle.</param>
        public void SetRotation(Quaternion rotation)
        {
            var inputAngles = rotation.eulerAngles;
            var rigidbodyAngles = rigidbody.rotation.eulerAngles;
            var pitch = ClampDegree360(rigidbodyAngles.x, maxSlope);
            var roll = ClampDegree360(rigidbodyAngles.z, maxSlope);
            rigidbody.MoveRotation(Quaternion.Euler(pitch, inputAngles.y, roll));
            var angularVelocity = rigidbody.angularVelocity;
            rigidbody.angularVelocity = new Vector3(angularVelocity.x, 0f, angularVelocity.z);

            static float ClampDegree360(float value, float maxAbsValue)
            {
                if (value < 360f - maxAbsValue && value > 180f)
                {
                    return 360f - maxAbsValue;
                }

                if (value > maxAbsValue && value <= 180f)
                {
                    return maxAbsValue;
                }

                return value;
            }
        }

        /// Set <see cref="TurnSignalState"/> and turn on/off blinkers according to the value of <see cref="TurnSignalState"/>.
        /// </summary>
        /// <param name="state">New value of <see cref="TurnSignalState"/>.</param>
        /// <exception cref="InvalidEnumArgumentException">Exception when <paramref name="state"/> is invalid value.</exception>
        public void SetTurnSignalState(TurnSignalState turnSignalState)
        {
            if (this.turnSignalState != turnSignalState)
                this.turnSignalState = turnSignalState;
        }

        /// <summary>
        /// Visual objects on/off
        /// </summary>
        /// <param name="isActive">visual on/off</param>
        public void SetActiveVisualObjects(bool isActive)
        {
            if (visualObjectRoot.activeSelf == isActive)
                return;

            visualObjectRoot.SetActive(isActive);
        }

        // Draw bounding box 
        private void OnDrawGizmos()
        {
            // Cache Gizmos default values.
            var cacheColor = Gizmos.color;
            var cacheMatrix = Gizmos.matrix;

            // Apply color and matrix.
            Gizmos.color = Color.white;
            Gizmos.matrix = Matrix4x4.TRS(transform.position, transform.rotation, transform.lossyScale);

            // Draw wire cube.
            Gizmos.DrawWireCube(bounds.center, bounds.size);

            // Return to default value.
            Gizmos.color = cacheColor;
            Gizmos.matrix = cacheMatrix;
        }
    }
}