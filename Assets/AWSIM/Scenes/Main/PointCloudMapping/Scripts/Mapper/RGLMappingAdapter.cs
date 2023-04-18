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
        [Tooltip("Resolution to sub-sample point cloud data. Set leaf size to 0 if you don't want to sub-sample.")]
        private float leafSize;

        private bool isInitialized = false;

        private LidarSensor lidarSensor;

        private RGLNodeSequence rglSubgraphMapping;

        private readonly string rosWorldTransformNodeId = "ROS_WORLD_TF";
        private readonly string downsampleNodeId = "DOWNSAMPLE";
        private readonly string temporalMergeNodeId = "TEMPORAL_MERGE";

        private string outputPcdFilePath;
        private bool downsamplingEnabled = false;

        public void Awake()
        {
            lidarSensor = GetComponent<LidarSensor>();
            // Make sure automatic capture in RGL Lidar Sensor is disabled.
            // We want to perform captures only on demand (after warping).
            lidarSensor.AutomaticCaptureHz = 0;
        }

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
                .AddNodePointsTransform(rosWorldTransformNodeId, worldTransform);

            if (leafSize > 0.0f)
            {
                rglSubgraphMapping.AddNodePointsDownsample(downsampleNodeId, new Vector3(leafSize, leafSize, leafSize));
                downsamplingEnabled = true;
            }

            rglSubgraphMapping.AddNodePointsTemporalMerge(temporalMergeNodeId, new RGLField[1] {RGLField.XYZ_F32});

            lidarSensor.ConnectToWorldFrame(rglSubgraphMapping);

            isInitialized = true;
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

            lidarSensor.Capture();

            if (downsamplingEnabled)
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
    }
}
