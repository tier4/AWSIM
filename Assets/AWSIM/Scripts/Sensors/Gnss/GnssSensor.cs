using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

namespace AWSIM
{
    /// <summary>
    /// GNSS sensor.
    /// Publish pose and poseWithCovarianceStamped in MGRS coordinate system.
    /// Need to set the MgrsReference of the Environment for the MGRS coordinate system.
    /// </summary>
    /// TODO: To improve the reproducibility of GNSS sensors, make it possible to output rotation and covariance.
    public class GnssSensor : MonoBehaviour
    {
        /// <summary>
        /// This data is output from GnssSensor at the OutputHz cycle.
        /// </summary>
        public class OutputData
        {
            /// <summary>
            /// Position in the MGRS coordinate system.
            /// (NOTE: This is the position considering the MGRS coordinate system origin set in Environment.cs,
            /// not the Unity world coordinate system position.)
            /// </summary>
            public Vector3 MgrsPosition;

            public OutputData()
            {
                MgrsPosition = new Vector3();
            }
        }

        /// <summary>
        /// Data output hz.
        /// Sensor processing and callbacks are called in this hz.
        /// </summary>
        [Range(0, 10)]
        public int OutputHz = 5;    // Autoware's GnssSensor basically output at 1hz.

        /// <summary>
        /// Delegate used in callbacks.
        /// </summary>
        /// <param name="outputData">Data output for each hz</param>
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

            // update mgrs position.
            var rosPosition = ROS2Utility.UnityToRosPosition(m_transform.position);
            outputData.MgrsPosition = rosPosition + Environment.Instance.MgrsOffsetPosition;   // ros gnss sensor's pos + mgrs offset pos.

            // Calls registered callbacks
            OnOutputData.Invoke(outputData);
        }
    }
}