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
            public Transform origin;
        }

        /// <summary>
        /// Data output hz.
        /// Sensor processing and callbacks are called in this hz.
        /// </summary>
        [Range(0, 10)]
        public int OutputHz = 10;    // Autoware's ObjectSensor basically output at 10hz.


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
        public OutputData outputData = new OutputData();
        private List<Classification> filteredObjects = new List<Classification>();
        private Classification[] cachedObjectsWithClassification;

        // This method generates a footprint for the vehicle based on its dimensions, position, and rotation.
        Vector2[] GenerateFootprint(Vector3 dimensions, Rigidbody vehicleRb)
        {
            Vector2[] footprint = new Vector2[4];

            // Retrieve the vehicle's position and rotation
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
                Vector3 globalCorner = rotation * localCorners[i];
                footprint[i] = new Vector2(globalCorner.x, globalCorner.z); // Using X and Z axes as Unity is left-handed and Y is up.
            }

            return footprint;
        }

        void CreateDetectedObjectData(){
            outputData.objects = new DetectedObject[cachedObjectsWithClassification.Length];
            for (int i = 0; i < cachedObjectsWithClassification.Length; i++) {
                Classification obj = cachedObjectsWithClassification[i];
                outputData.objects[i] = new DetectedObject();
                // add classification
                outputData.objects[i].classification = obj.objectType;
                var gameObject = obj.gameObject;
                // Note: object without rigidbody is considered above
                var rb = gameObject.GetComponent<Rigidbody>();
                if(rb == null){
                    Debug.Log("Please Attach RigidBody to NPC");
                }
                outputData.objects[i].rigidBody = rb;
                MeshFilter[] meshFilters = gameObject.GetComponentsInChildren<MeshFilter>();
                if(meshFilters.Length > 0){
                    Vector3 localMinBounds = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
                    Vector3 localMaxBounds = new Vector3(float.MinValue, float.MinValue, float.MinValue);
                    // mesh filter bounds is in local coordinate
                    foreach (MeshFilter meshFilter in meshFilters)
                    {
                        Bounds localBounds = meshFilter.sharedMesh.bounds;
                        localMinBounds = Vector3.Min(localMinBounds, localBounds.min);
                        localMaxBounds = Vector3.Max(localMaxBounds, localBounds.max);
                    }
                    outputData.objects[i].dimension = ROS2Utility.UnityToRosScale(localMaxBounds - localMinBounds);
                    outputData.objects[i].bounds = GenerateFootprint(outputData.objects[i].dimension, outputData.objects[i].rigidBody);
                } else {
                    outputData.objects[i].dimension = new Vector3(0.5f, 0.5f, 1.5f);
                    outputData.objects[i].bounds = new Vector2[]{};
                }                
                i++;
            }            
        }

        void Start()
        {
            outputData.origin = this.transform;
            cachedObjectsWithClassification = FindObjectsOfType<Classification>();
            CreateDetectedObjectData();
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
            outputData.origin = this.transform;
            var currentObjectsWithClassification = FindObjectsOfType<Classification>();
            if (!Enumerable.SequenceEqual(cachedObjectsWithClassification, currentObjectsWithClassification)) {
                cachedObjectsWithClassification = currentObjectsWithClassification;
                CreateDetectedObjectData();
            }
            for (int i = 0; i < cachedObjectsWithClassification.Length; i++)
            {
                var o = outputData.objects[i];
                if(o == null) continue;
                outputData.objects[i].bounds = GenerateFootprint(o.dimension,o.rigidBody);
            }

            // Calls registered callbacks
            OnOutputData.Invoke(outputData);
        }
    }
}