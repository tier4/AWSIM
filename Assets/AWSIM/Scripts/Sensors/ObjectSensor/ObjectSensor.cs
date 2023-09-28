using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using System.Linq;

namespace AWSIM
{
    /// <summary>
    /// ObjectSensor sensor.
    /// Need to set the MgrsReference of the Environment for the MGRS coordinate system.
    /// </summary>
    /// TODO: To improve the reproducibility of ObjectSensor sensors, make it possible to output rotation and covariance.
    public class ObjectSensor : MonoBehaviour
    {
        /// <summary>
        /// This data is output from ObjectSensor at the OutputHz cycle.
        /// </summary>
        /// 

        public Classification classification;
        
        public class DetectedObject
        {
            public Rigidbody rigidBody;
            public Vector3 dimension;
            public Vector2[] bounds;
            public Classification.ObjectType classification; 
        }

        public class OutputData
        {
            public DetectedObject[] objects;
        }

        /// <summary>
        /// Data output hz.
        /// Sensor processing and callbacks are called in this hz.
        /// </summary>
        [Range(0, 10)]
        public int OutputHz = 10;    // Autoware's ObjectSensor basically output at 10hz.
        [Range(0, 200)]
        public float maxDistance = 200f;

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
        GameObject[] gameObjects;

        // This method generates a footprint for the vehicle based on its dimensions, position, and rotation.
        Vector2[] GenerateFootprint(Vector3 dimensions, Rigidbody vehicleRb)
        {
            Vector2[] footprint = new Vector2[4];

            // Retrieve the vehicle's position and rotation
            Vector3 position = vehicleRb.position;
            Quaternion rotation = vehicleRb.rotation;

            // Calculate half of the dimensions for easier calculation
            float halfLength = dimensions.x * 0.5f;
            float halfWidth = dimensions.y * 0.5f;

            // Define the vehicle's corner points relative to its center
            Vector3[] localCorners = {
                new Vector3(-halfLength, 0, halfWidth),    // Front left
                new Vector3(-halfLength, 0, -halfWidth),   // Rear left
                new Vector3(halfLength, 0, -halfWidth),    // Rear right
                new Vector3(halfLength, 0, halfWidth)      // Front right
            };

            // Rotate and translate each corner point to get the footprint
            for (int i = 0; i < 4; i++)
            {
                Vector3 globalCorner = position + rotation * localCorners[i];
                footprint[i] = new Vector2(globalCorner.x, globalCorner.z); // Using X and Z axes as Unity is left-handed and Y is up.
            }

            return footprint;
        }

        void Start()
        {
            m_transform = transform;
            Classification[] objectsWithClassification = FindObjectsOfType<Classification>();
            List<Classification> filteredObjects = new List<Classification>();
            outputData = new OutputData();
            int i = 0;
            // limit to possible detected object 
            foreach (Classification obj in objectsWithClassification)
            {
                float distance = Vector3.Distance(transform.position, obj.transform.position);
                if (distance < maxDistance && obj.gameObject.GetComponent<Rigidbody>() != null)
                {
                    filteredObjects.Add(obj);
                }
            }
            outputData.objects = new DetectedObject[filteredObjects.Count];
            foreach (Classification obj in filteredObjects)
            {
                outputData.objects[i] = new DetectedObject();
                Debug.Log(obj.objectType);
                // add classification
                outputData.objects[i].classification = obj.objectType;
                Debug.Log(obj.gameObject.name);
                var gameObject = obj.gameObject;
                // Note: object without rigidbody is considered above
                var rb = gameObject.GetComponent<Rigidbody>();
                outputData.objects[i].rigidBody = rb;
                Vector3 minBounds = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
                Vector3 maxBounds = new Vector3(float.MinValue, float.MinValue, float.MinValue);
                MeshRenderer[] renderers = gameObject.GetComponentsInChildren<MeshRenderer>();
                foreach (MeshRenderer renderer in renderers)
                {
                    Bounds bounds = renderer.bounds;
                    minBounds = Vector3.Min(minBounds, bounds.min);
                    maxBounds = Vector3.Max(maxBounds, bounds.max);
                }
                // add dimension
                Vector3 totalSize = maxBounds - minBounds;
                outputData.objects[i].dimension = totalSize;
                outputData.objects[i].bounds = GenerateFootprint(totalSize,outputData.objects[i].rigidBody);
                i++;

            }
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
            for (int i = 0; i < outputData.objects.Length; i++)
            {
                var o = outputData.objects[i];
                outputData.objects[i].bounds = GenerateFootprint(o.dimension,o.rigidBody);
            }

            // Calls registered callbacks
            OnOutputData.Invoke(outputData);
        }
    }
}