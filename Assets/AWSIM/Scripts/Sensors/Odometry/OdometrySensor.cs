using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// Outputs the true value of odometry.
    /// TODO: Currently the Mgrs coordinate system is output, but refactor the Environment to Unity coordinate system + offset.
    /// </summary>
    public class OdometrySensor : MonoBehaviour
    {
        /// <summary>
        /// This data is output from OdometrySensor at the OutputHz cycle.
        /// </summary>
        public class OutputData
        {
            public Vector3 Position;
            public Quaternion Rotation;
            public Vector3 linearVelocity;
            public Vector3 angularVelocity;


            public OutputData()
            {
                Position = new Vector3();
                Rotation = new Quaternion();
                linearVelocity = new Vector3();
                angularVelocity = new Vector3();
            }
        }

        /// <summary>
        /// Data output hz.
        /// Sensor processing and callbacks are called in this hz.
        /// </summary>
        [Range(0, 100)]
        public int OutputHz = 60;       // Set default the same as fixed update hz.
        Vector3 lastPosition;           // Previous frame position used for acceleration calculation.
        Vector3 lastLocalVelocity;      // Previous frame velocity used for acceleration calculation.
        QuaternionD lastRotation;       // Previous frame rotation used for angular velocity calculation.

        /// <summary>
        /// Delegate used in callbacks.
        /// </summary>
        public delegate void OnOutputDataDelegate(OutputData outputData);

        /// <summary>
        /// Called each time data is output.
        /// </summary>
        public OnOutputDataDelegate OnOutputData;

        float timer = 0;
        OutputData outputData = new OutputData();
        Transform m_transform;

        void Start()
        {
            m_transform = transform;
            lastPosition = transform.position;
            lastRotation = new QuaternionD(transform.rotation);
        }

        void FixedUpdate()
        {
            // Matching output to hz.
            timer += Time.deltaTime;
            var interval = 1.0f / OutputHz;
            interval -= 0.00001f;       // Allow for accuracy errors.
            if (timer < interval)
                return;
            timer = 0;

            // update ground truth position and rotation.
            var rosPosition = ROS2Utility.UnityToRosPosition(m_transform.position);
            outputData.Position = rosPosition + Environment.Instance.MgrsOffsetPosition;     // TODO: Handled in Unity coordinate system
                                                                                             // ros gnss sensor's pos + mgrs offset pos.

            var rosRotation = ROS2Utility.UnityToRosRotation(m_transform.rotation);
            outputData.Rotation = rosRotation;

            // Compute angular velocity.
            var currentRotation = new QuaternionD(transform.rotation);
            var deltaRotation = currentRotation * QuaternionD.Inverse(lastRotation);
            deltaRotation.ToAngleAxis(out var angle, out var axis);
            var angularVelocity = (1.0f / Time.deltaTime) * (float)angle * axis;
            var localAngularVelocity = transform.InverseTransformDirection(angularVelocity);
            lastRotation = currentRotation;

            // Compute local velocity.
            var localVelocity = (transform.InverseTransformDirection(transform.position - lastPosition)) / Time.deltaTime;
            lastPosition = transform.position;

            // TODO: Temporarily avoid NaN values. Needs investigation.
            if (float.IsNaN(localAngularVelocity.x) || float.IsNaN(localAngularVelocity.y) || float.IsNaN(localAngularVelocity.z))
                localAngularVelocity = Vector3.zero;

            // Update output data.
            outputData.linearVelocity = localVelocity;
            outputData.angularVelocity = localAngularVelocity;

             // Calls registered callbacks
            OnOutputData.Invoke(outputData);
        }
    }
}