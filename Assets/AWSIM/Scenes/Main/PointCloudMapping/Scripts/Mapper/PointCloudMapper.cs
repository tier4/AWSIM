using System;
using AWSIM.Lanelet;
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

        private RGLMappingAdapter mappingSensor;
        private Queue<Pose> capturePoseQueue;

        private void Start()
        {
            // TODO: Support multiple sensors
            mappingSensor = vehicleGameObject.GetComponentInChildren<RGLMappingAdapter>();
            if (mappingSensor == null)
            {
                Debug.LogError($"Could not find mapping sensor in {vehicleGameObject.name}. Disabling PointCloudMapper!");
                enabled = false;
                return;
            }

            Debug.Log($"Found mapping sensor in {vehicleGameObject.name}: {mappingSensor.GetSensorName()}");

            mappingSensor.Initialize(worldOriginROS, $"{Application.dataPath}/{outputPcdFilePath}");

            var laneletMap = new OsmToLaneletMap(worldOriginROS).Convert(osmContainer.Data);

            var start = Time.realtimeSinceStartup;
            capturePoseQueue = new Queue<Pose>(LaneletMapToPoses(laneletMap, captureLocationInterval));
            var computeTimeMs = (Time.realtimeSinceStartup - start) * 1000f; 
            Debug.Log($"Will visit {capturePoseQueue.Count} points; computed in {computeTimeMs} ms");

            laneletVisualizer.Initialize(laneletMap);
            laneletVisualizer.CreateCenterline(transform);
        }

        private void Update()
        {
            Debug.Log($"PointCloudMapper: {capturePoseQueue.Count} captures left");
            if (capturePoseQueue.Count == 0)
            {
                SavePcd();
                enabled = false;
                return;
            }

            var currentPose = capturePoseQueue.Dequeue();
            vehicleGameObject.transform.position = currentPose.position;
            vehicleGameObject.transform.rotation = currentPose.rotation;

            mappingSensor.Capture();
        }

        public void OnDestroy()
        {
            if (enabled)
            {
                SavePcd();
            }
            mappingSensor.Destroy();
        }

        private void SavePcd()
        {
            Debug.Log($"Writing PCD to {Application.dataPath}/{outputPcdFilePath}");
            mappingSensor.SavePcd();
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
