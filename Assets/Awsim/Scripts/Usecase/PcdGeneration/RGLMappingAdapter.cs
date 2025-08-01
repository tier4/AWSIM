// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using UnityEngine;
using RGLUnityPlugin;

namespace Awsim.Usecase.PcdGeneration
{
    [RequireComponent(typeof(LidarSensor))]
    public class RGLMappingAdapter : MonoBehaviour
    {
        const string _rosWorldTransformNodeId = "ROS_WORLD_TF";
        const string _downsampleNodeId = "DOWNSAMPLE";
        const string _temporalMergeNodeId = "TEMPORAL_MERGE";

        [SerializeField]
        [Tooltip("Enable/disable point cloud data downsampling")]
        bool _enableDownsampling;

        [SerializeField]
        [Tooltip("Resolution of point cloud data downsampling")]
        [Min(0.000001f)]
        float _leafSize;

        string _outputPcdFilePath = string.Empty;
        bool _initialized = false;
        LidarSensor _lidarSensor = null;
        RGLNodeSequence _rglSubgraphMapping = null;

        public void Initialize(Vector3 worldOriginRos, string outputPcdFilePath)
        {
            if (_initialized)
            {
                throw new Exception("Attempted to initialize RGLMappingAdapter twice!");
            }

            _lidarSensor = GetComponent<LidarSensor>();
            _outputPcdFilePath = outputPcdFilePath;

            // Create and connect subgraph
            Matrix4x4 worldTransform = ROS2.Transformations.Unity2RosMatrix4x4();
            worldTransform.SetColumn(3, worldTransform.GetColumn(3) + (Vector4)worldOriginRos);
            _rglSubgraphMapping = new RGLNodeSequence()
                .AddNodePointsTransform(_rosWorldTransformNodeId, worldTransform)
                .AddNodePointsDownsample(_downsampleNodeId, new Vector3(_leafSize, _leafSize, _leafSize))
                .AddNodePointsTemporalMerge(_temporalMergeNodeId, new RGLField[1] { RGLField.XYZ_VEC3_F32 });

            _rglSubgraphMapping.SetActive(_downsampleNodeId, _enableDownsampling);
            _lidarSensor.ConnectToWorldFrame(_rglSubgraphMapping);
            _initialized = true;
        }

        public void SavePcd()
        {
            if (_rglSubgraphMapping == null)
            {
                Debug.LogWarning("RGLMappingAdapter: skipped saving PCD file - empty point cloud");
                return;
            }

            _rglSubgraphMapping.SavePcdFile(_outputPcdFilePath);
        }

        public void Capture()
        {
            if (!_initialized)
            {
                throw new Exception("Attempted to run RGLMappingAdapter without initialization!");
            }

            if (!enabled)
            {
                return;
            }

            _lidarSensor.Capture();

            if (_enableDownsampling)
            {
                int countBeforeDownsample = _rglSubgraphMapping.GetPointCloudCount(_rosWorldTransformNodeId);
                int countAfterDownsample = _rglSubgraphMapping.GetPointCloudCount(_downsampleNodeId);
                bool pointCloudReduced = countAfterDownsample < countBeforeDownsample;
                if (!pointCloudReduced)
                {
                    Debug.LogWarning($"Downsampling had no effect for '{name}'. If you see this message often, consider increasing leafSize.");
                }
            }
        }

        public void Dispose()
        {
            _rglSubgraphMapping.Clear();
            _initialized = false;
        }

        void Reset()
        {
            _lidarSensor = GetComponent<LidarSensor>();
            _lidarSensor.AutomaticCaptureHz = 0;
        }

        void OnValidate()
        {
            if (_rglSubgraphMapping == null)
                return;

            _rglSubgraphMapping.UpdateNodePointsDownsample(_downsampleNodeId, new Vector3(_leafSize, _leafSize, _leafSize));
            _rglSubgraphMapping.SetActive(_downsampleNodeId, _enableDownsampling);
        }
    }
}
