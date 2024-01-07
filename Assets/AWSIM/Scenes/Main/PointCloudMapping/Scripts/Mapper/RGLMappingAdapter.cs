// Copyright 2022 Robotec.ai.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using RGLUnityPlugin;
using UnityEngine;

namespace AWSIM.PointCloudMapping
{
    /// <summary>
    /// Implementation of an adapter to LidarSensor based on RGL for PointCloudMapper.
    /// </summary>
    [RequireComponent(typeof(LidarSensor))]
    public class RGLMappingAdapter : MonoBehaviour
    {
        [SerializeField]
        [Tooltip("Enable/disable point cloud data downsampling")]
        private bool enableDownsampling;

        [SerializeField]
        [Tooltip("Resolution of point cloud data downsampling")]
        [Min(0.000001f)]
        private float leafSize;

        private bool isInitialized = false;

        private LidarSensor lidarSensor;

        private RGLNodeSequence rglSubgraphMapping;

        private readonly string rosWorldTransformNodeId = "ROS_WORLD_TF";
        private readonly string downsampleNodeId = "DOWNSAMPLE";
        private readonly string temporalMergeNodeId = "TEMPORAL_MERGE";

        private string outputPcdFilePath;

        public void Awake()
        {
            lidarSensor = GetComponent<LidarSensor>();
            // Make sure automatic capture in RGL Lidar Sensor is disabled.
            // We want to perform captures only on demand (after warping).
            lidarSensor.AutomaticCaptureHz = 0;
        }

        // Empty function to make enable checkbox appear in the Inspector
        public void Start() {}

        public void Initialize(Vector3 worldOriginROS, string outputPcdFilePath)
        {
            if (isInitialized)
            {
                throw new Exception("Attempted to initialize RGLMappingAdapter twice!");
            }

            this.outputPcdFilePath = outputPcdFilePath;

            // Create and connect subgraph
            Matrix4x4 worldTransform = ROS2.Transformations.Unity2RosMatrix4x4();
            worldTransform.SetColumn(3, worldTransform.GetColumn(3) + (Vector4) worldOriginROS);
            rglSubgraphMapping = new RGLNodeSequence()
                .AddNodePointsTransform(rosWorldTransformNodeId, worldTransform)
                .AddNodePointsDownsample(downsampleNodeId, new Vector3(leafSize, leafSize, leafSize))
                .AddNodePointsTemporalMerge(temporalMergeNodeId, new RGLField[1] {RGLField.XYZ_VEC3_F32});

            rglSubgraphMapping.SetActive(downsampleNodeId, enableDownsampling);

            lidarSensor.ConnectToWorldFrame(rglSubgraphMapping);

            isInitialized = true;
        }

        public void OnValidate()
        {
            if (rglSubgraphMapping == null)
            {
                return;
            }

            rglSubgraphMapping.UpdateNodePointsDownsample(downsampleNodeId, new Vector3(leafSize, leafSize, leafSize));
            rglSubgraphMapping.SetActive(downsampleNodeId, enableDownsampling);
        }

        public string GetSensorName()
        {
            return gameObject.name;
        }

        public void SavePcd()
        {
            if (rglSubgraphMapping == null)
            {
                Debug.LogWarning("RGLMappingAdapter: skipped saving PCD file - empty point cloud");
                return;
            }
            rglSubgraphMapping.SavePcdFile(outputPcdFilePath);
        }

        public void Capture()
        {
            if (!isInitialized)
            {
                throw new Exception("Attempted to run RGLMappingAdapter without initialization!");
            }

            if (!enabled)
            {
                return;
            }

            lidarSensor.Capture();

            if (enableDownsampling)
            {
                int countBeforeDownsample = rglSubgraphMapping.GetPointCloudCount(rosWorldTransformNodeId);
                int countAfterDownsample = rglSubgraphMapping.GetPointCloudCount(downsampleNodeId);
                bool pointCloudReduced = countAfterDownsample < countBeforeDownsample;
                if (!pointCloudReduced)
                {
                    Debug.LogWarning($"Downsampling had no effect for '{name}'. If you see this message often, consider increasing leafSize.");
                }
            }
        }

        // Called in PointCloudMapper.OnDestroy()
        // Must be executed after destroying PointCloudMapper because SavePcd is called there
        // To be refactored when implementing multi-sensor mapping
        public void Destroy()
        {
            rglSubgraphMapping.Clear();
            isInitialized = false;
        }
    }
}
