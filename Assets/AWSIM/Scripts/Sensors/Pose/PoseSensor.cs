using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    public class PoseSensor : MonoBehaviour
    {
        public class OutputData
        {
            public Vector3 GroundTruthPosition;
            public Quaternion GrountTruthRotation;
        }

        [Range(0, 100)]
        public int OutputHz = 100;

        public delegate void OnOutputDataDelegate(OutputData outputData);

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

            // update ground truth position and rotation.
            var rosPosition = ROS2Utility.UnityToRosPosition(m_transform.position);
            outputData.GroundTruthPosition = rosPosition + Environment.Instance.MgrsOffsetPosition;     // TODO: Handled in Unity coordinate system
                                                                                                        // ros gnss sensor's pos + mgrs offset pos.

            var rosRotation = ROS2Utility.UnityToRosRotation(m_transform.rotation);
            outputData.GrountTruthRotation = rosRotation;
            
             // Calls registered callbacks
            OnOutputData.Invoke(outputData);
        }
    }
}