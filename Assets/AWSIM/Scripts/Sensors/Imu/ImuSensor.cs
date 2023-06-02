using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// IMU (Inertial Measurement Unit) Sensor.
    /// Measure the Acceleration(m/s^2) and AngularVelocity(rad/s) based on the Transform of the GameObject to which this component is attached, and publish it to ROS2.
    /// </summary>
    public class ImuSensor : MonoBehaviour
    {
        /// <summary>
        /// This data is output from ImuSensor at the OutputHz cycle.
        /// </summary>
        public class OutputData
        {
            /// <summary>
            /// Measured acceleration(m/s^2)
            /// </summary>
            public Vector3 LinearAcceleration;

            /// <summary>
            /// Measured angular velocity(rad/s)
            /// </summary>
            public Vector3 AngularVelocity;

            public OutputData()
            {
                LinearAcceleration = new Vector3();
                AngularVelocity = new Vector3();
            }
        }

        /// <summary>
        /// Data output hz.
        /// Sensor processing and callbacks are called in this hz.
        /// </summary>
        [Range(1, 100)]
        public int OutputHz = 30;   // Autoware's ImuSensor basically output at 30hz.

        /// <summary>
        /// Delegate used in callbacks.
        /// </summary>
        /// <param name="outputData">Data output for each hz</param>
        public delegate void OnOutputDataDelegate(OutputData outputData);

        /// <summary>
        /// Called each time data is output.
        /// </summary>
        public OnOutputDataDelegate OnOutputData;

        /// <summary>
        /// The bool value determines whether to gravity is considered or not
        ///</summary>
        public bool EnableGravity;


        Vector3 lastPosition;           // Previous frame position used for acceleration calculation.
        Vector3 lastVelocity;           // Previous frame velocity used for acceleration calculation in global coordinate system.
        Vector3 lastLocalVelocity;      // Previous frame velocity used for acceleration calculation.
        QuaternionD lastRotation;       // Previous frame rotation used for angular velocity calculation.
        float timer = 0;
        OutputData outputData = new OutputData();

        Vector3 g;                      // Gravity considered in measuring of acceleration and angular velocity

        void Start()
        {
            lastRotation = new QuaternionD(transform.rotation);
            lastPosition = transform.position;

            if (EnableGravity == true)
            {
                g = Physics.gravity;
            }
            else
            {
                g = Vector3.zero;
            }
        }

        void FixedUpdate()
        {
            // Update timer.
            timer += Time.deltaTime;

            // Compute angular velocity.
            var currentRotation = new QuaternionD(transform.rotation);
            var deltaRotation = currentRotation * QuaternionD.Inverse(lastRotation);
            deltaRotation.ToAngleAxis(out var angle, out var axis);
            var angularVelocity = (1.0f / Time.deltaTime) * (float)angle * axis;
            var localAngularVelocity = transform.InverseTransformDirection(angularVelocity);
            lastRotation = currentRotation;

            // Compute acceleration.
            var Velocity = (transform.position - lastPosition) / Time.deltaTime;
            var localVelocity = (transform.InverseTransformDirection(transform.position - lastPosition)) / Time.deltaTime;
            var localAcceleration = transform.InverseTransformDirection((Velocity - lastVelocity) / Time.deltaTime + g);
            lastPosition = transform.position;
            lastVelocity = Velocity;
            lastLocalVelocity = localVelocity;

            // Matching output to hz.
            var interval = 1.0f / OutputHz;
            interval -= 0.00001f;       // Allow for accuracy errors.
            if (timer < interval)
                return;
            timer = 0;

            // TODO: Temporarily avoid NaN values. Needs investigation.
            if (float.IsNaN(localAngularVelocity.x) || float.IsNaN(localAngularVelocity.y) || float.IsNaN(localAngularVelocity.z))
                localAngularVelocity = Vector3.zero;

            // Update output data.
            outputData.LinearAcceleration = localAcceleration;
            outputData.AngularVelocity = localAngularVelocity;

            // Calls registered callbacks
            OnOutputData.Invoke(outputData);

        }
    }
}