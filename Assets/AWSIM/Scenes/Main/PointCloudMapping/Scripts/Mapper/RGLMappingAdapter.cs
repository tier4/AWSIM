using System;
using RGLUnityPlugin;
using UnityEngine;

namespace AWSIM.PointCloudMapping
{
    /// <summary>
    /// Implementation of IMappingSensor for PointCloudMapper based on RGL.
    /// Provides a filtered PCL based on the visualization points for Unity.
    /// Intensity is dummy.
    /// </summary>
    public class RGLMappingAdapter : MonoBehaviour
    {
        [SerializeField]
        [Tooltip("Resolution to sub-sample point cloud data. Set leaf size to 0 if you don't want to sub-sample.")]
        private float leafSize;

        private Vector3 worldOriginROS;
        private string outputPCDFilePath = "output.pcd";

        private LidarSensor lidarSensor;

        private RGLNodeSequence rglGraphMapping;

        private string rosWorldTransformNodeId = "ROS_WORLD_TF";
        private string downsampleNodeId = "DOWNSAMPLE";
        private string writePcdNodeId = "WRITE_PCD";

        public void Awake()
        {
            rglGraphMapping = new RGLNodeSequence()
                .AddNodePointsTransform(rosWorldTransformNodeId, ROS2.Transformations.Unity2RosMatrix4x4())
                .AddNodePointsDownsample(downsampleNodeId, new Vector3(1, 1, 1))
                .AddNodePointsWritePCDFile(writePcdNodeId, outputPCDFilePath);
        }

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

            if (leafSize > 0.0f)
            {
                rglGraphMapping.UpdateNodePointsDownsample(downsampleNodeId, new Vector3(leafSize, leafSize, leafSize));
            }
            else
            {
                rglGraphMapping.RemoveNode(downsampleNodeId);
            }

            lidarSensor.ConnectToWorldFrame(rglGraphMapping);
        }

        public string GetSensorName()
        {
            return gameObject.name;
        }

        public void SetOutputPcdFilePath(string path)
        {
            outputPCDFilePath = path;
            rglGraphMapping.UpdateNodePointsWritePCDFile(writePcdNodeId, outputPCDFilePath);
        }

        public void SetWorldOriginROS(Vector3 position)
        {
            worldOriginROS = position;
            Matrix4x4 transform = ROS2.Transformations.Unity2RosMatrix4x4();
            transform.SetColumn(3, transform.GetColumn(3) + (Vector4)worldOriginROS);
            rglGraphMapping.UpdateNodePointsTransform(rosWorldTransformNodeId, transform);
        }

        public void SavePcd()
        {
            rglGraphMapping.Clear();
        }

        public void Capture()
        {
            lidarSensor.Capture();
        }
    }
}
