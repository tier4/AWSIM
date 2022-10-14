using PclSharp;
using PclSharp.Struct;
using RGLUnityPlugin;
using UnityEngine;

namespace AWSIM.PointCloudMapping
{
    /// <summary>
    /// Implementation of IMappingSensor for PointCloudMapper based on RGL.
    /// Provides a filtered PCL based on the visualization points for Unity.
    /// Intensity is dummy.
    /// </summary>
    public class RGLMappingAdapter : MonoBehaviour, IMappingSensor
    {
        [SerializeField]
        [Tooltip("Resolution to sub-sample point cloud data. Set leaf size to 0 if you don't want to sub-sample.")]
        private float leafSize;
        
        private LidarSensor lidarSensor;

        public void Start()
        {
            if (!TryGetComponent(out lidarSensor) || !lidarSensor.enabled)
            {
                Debug.LogError("LidarSensor not found for RGLMappingAdapter. Make sure it is attached to the same GameObject!");
                enabled = false;
                return;
            }
            // Make sure automatic capture in RGL Lidar Sensor is disabled.
            // We want to perform captures only on demand (after warping).
            lidarSensor.AutomaticCaptureHz = 0;
        }

        public string GetSensorName()
        {
            return gameObject.name;
        }

        public PointCloudOfXYZI Capture_XYZI_ROS(Vector3 worldOriginROS)
        {
            var outputData = lidarSensor.RequestCapture();
            var pcl = new PointCloudOfXYZI();

            for (int i = 0; i < outputData.hitCount; ++i)
            {
                var rosPoint = ROS2Utility.UnityToRosPosition(outputData.hits[i]) + worldOriginROS;
                pcl.Add(new PointXYZI
                {
                    X = rosPoint.x,
                    Y = rosPoint.y,
                    Z = rosPoint.z,
                    Intensity = 100.0f
                });
            }

            if (leafSize == 0.0f)
            {
                return pcl;
            }

            var filteredPCL = new PointCloudOfXYZI();
            var voxelGrid = new PclSharp.Filters.VoxelGridOfXYZI();
            voxelGrid.SetInputCloud(pcl);
            voxelGrid.LeafSize = new PointXYZ { V = new System.Numerics.Vector3(leafSize, leafSize, leafSize) };
            voxelGrid.filter(filteredPCL);
            return filteredPCL;
        }
    }
}
