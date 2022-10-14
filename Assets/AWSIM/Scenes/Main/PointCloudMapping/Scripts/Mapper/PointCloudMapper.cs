using System;
using AWSIM.Lanelet;
using PclSharp;
using System.Collections.Generic;
using AWSIM.PointCloudMapping.Geometry;
using UnityEngine;

namespace AWSIM.PointCloudMapping
{
    /// <summary>
    /// Provide functionality to conduct point cloud mapping along all centerlines in OSM.
    /// If you play your scene, PointCloudMapper will automatically start mapping.
    /// The vehicle keeps warping along centerlines at a interval of <see cref="captureLocationInterval"/> and point cloud data from sensors are captured at every warp point.
    /// PCD file will be outputted when you stop your scene or all locations in the route are captured.
    /// </summary>
    public class PointCloudMapper : MonoBehaviour
    {
        [SerializeField]
        [Tooltip("Imported OSM file. Mapping is conducted along all centerlines of lanelets in the OSM.")]
        private OsmDataContainer osmContainer;

        [SerializeField]
        [Tooltip("Game object containing sensors to capture pointcloud. It will be warped along centerlines of lanelets.")]
        private GameObject vehicleGameObject;

        [SerializeField]
        [Tooltip("Result PCD file name. On Editor/Windows, it will be saved in Assets/")]
        private string outputPcdFilePath = "output.pcd";

        [SerializeField]
        [Tooltip("Distance in meters between consecutive warps along the centerline of a lanelet.")]
        private float captureLocationInterval = 6f;

        [SerializeField]
        [Tooltip("World origin in ROS coordinate systems, will be added to every point coordinates")]
        private Vector3 worldOriginROS;

        [SerializeField]
        [Tooltip("Configurable visualization of the loaded lanelet map")]
        private LaneletVisualizer laneletVisualizer;

        private List<IMappingSensor> mappingSensors;
        private Queue<Pose> capturePoseQueue;
        private PointCloudOfXYZI mergedPCL;

        public void Start()
        {
            mappingSensors = new List<IMappingSensor>(vehicleGameObject.GetComponentsInChildren<IMappingSensor>());
            if (mappingSensors.Count == 0)
            {
                Debug.LogError($"Found 0 sensors in {vehicleGameObject.name}. Disabling PointCloudMapper!");
                enabled = false;
                return;
            }

            Debug.Log($"Found {mappingSensors.Count} sensors in {vehicleGameObject.name}:");
            foreach (var mappingSensor in mappingSensors)
            {
                Debug.Log($"- IMappingSensor: {mappingSensor.GetSensorName()}");
            }
            
            var laneletMap = new OsmToLaneletMap(worldOriginROS).Convert(osmContainer.Data);
            
            var start = Time.realtimeSinceStartup;
            capturePoseQueue = new Queue<Pose>(LaneletMapToPoses(laneletMap, captureLocationInterval));
            var computeTimeMs = (Time.realtimeSinceStartup - start) * 1000f; 
            Debug.Log($"Will visit {capturePoseQueue.Count} points; computed in {computeTimeMs} ms");
            
            laneletVisualizer.Initialize(laneletMap);
            laneletVisualizer.CreateCenterline(transform);
            
            mergedPCL = new PointCloudOfXYZI();
        }

        public void Update()
        {
            Debug.Log($"PointCloudMapper: {capturePoseQueue.Count} captures left");
            if (capturePoseQueue.Count == 0)
            {
                SavePCL();
                enabled = false;
                return;
            }

            var currentPose = capturePoseQueue.Dequeue();
            vehicleGameObject.transform.position = currentPose.position;
            vehicleGameObject.transform.rotation = currentPose.rotation;

            foreach (var sensor in mappingSensors)
            {
                var sensorPCL = sensor.Capture_XYZI_ROS(worldOriginROS);
                foreach (var point in sensorPCL.Points)
                {
                    mergedPCL.Add(point);
                }
            }
        }

        public void OnDestroy()
        {
            if (enabled)
            {
                SavePCL();
            }
        }

        private void SavePCL()
        {
            var path = $"{Application.dataPath}/{outputPcdFilePath}";
            Debug.Log($"Writing PCL data to {path}");
            var writer = new PclSharp.IO.PCDWriter();
            writer.Write(path, mergedPCL);
            Debug.Log("PCL data saved successfully");
        }

        private static IEnumerable<Pose> LaneletMapToPoses(LaneletMap laneletMap, float jumpDistance)
        {
            foreach (var laneletData in laneletMap.Lanelets.Values)
            {
                float distanceVisited = 0.0f;
                Vector3[] centerPoints = laneletData.CalculateCenterline();
                BezierPath bezierPath = new BezierPathFactory().CreateBezierPath(centerPoints);

                while (distanceVisited <= bezierPath.Length)
                {
                    yield return bezierPath.TangentPose(distanceVisited);
                    distanceVisited += jumpDistance;
                }
            }
        }
    }
}
