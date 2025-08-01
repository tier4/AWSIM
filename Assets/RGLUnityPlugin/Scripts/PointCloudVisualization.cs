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
using System.Collections.Generic;
using UnityEngine;

namespace RGLUnityPlugin
{
    [System.Serializable]
    public class PointCloudVisualization : MonoBehaviour
    {
        public enum PointShape
        {
            FlatSquare = 0,
            Box = 1,
            Pyramid = 2
        }

        private static readonly List<Color> rainbowColors = new List<Color> {
            Color.red,
            new Color(1, 0.5f, 0, 1), // orange
            Color.yellow,
            Color.green,
            Color.blue,
            new Color(0.5f, 0, 1, 1) // violet
        };

        [SerializeField]
        private PointShape pointShape = PointShape.Box;

        [SerializeField]
        [Range(0.005f, 0.5f)]
        private float pointSize = 0.05f;

        [SerializeField]
        private List<Color> colors = new List<Color>(rainbowColors);

        [SerializeField]
        private bool autoComputeColoringHeights = false;

        [SerializeField]
        private float minColoringHeight = 0f;

        [SerializeField]
        private float maxColoringHeight = 20f;

        private Material material = null;

        private static readonly int visualizationLayerID = 11;

        private Mesh mesh;

        private Vector3[] onlyHits = Array.Empty<Vector3>();
        private int pointCount = 0;
        private int[] indices = Array.Empty<int>();

        private RGLNodeSequence rglSubgraphVisualizationOutput;
        private const string visualizationOutputNodeId = "OUT_VISUALIZATION";

        private MonoBehaviour sensor;

        public void Awake()
        {
            rglSubgraphVisualizationOutput = new RGLNodeSequence()
                .AddNodePointsYield(visualizationOutputNodeId, RGLField.XYZ_VEC3_F32);

            rglSubgraphVisualizationOutput.SetPriority(visualizationOutputNodeId, 1);
        }

        public void Start()
        {
            // Check if LiDAR is attached
            var lidar = GetComponent<LidarSensor>();
            if (lidar != null)
            {
                lidar.ConnectToWorldFrame(rglSubgraphVisualizationOutput);
                lidar.onNewData += OnNewLidarData;
                sensor = lidar;
            }

            // Check if radar is attached
            var radar = GetComponent<RadarSensor>();
            if (radar != null)
            {
                if (sensor != null)
                {
                    Debug.LogError($"More than one sensor is attached to the PointCloudVisualization. Destroying {name}.");
                    Destroy(this);
                    return;
                }
                radar.ConnectToWorldFrame(rglSubgraphVisualizationOutput);
                radar.onNewData += OnNewLidarData;
                sensor = radar;
            }

            if (sensor == null)
            {
                Debug.LogError($"Cannot visualize point cloud without sensor. Destroying {name}.");
                Destroy(this);
                return;
            }

            mesh = new Mesh();
            if (!material)
            {
                material = Instantiate<Material>(Resources.Load("PointCloudMaterial", typeof(Material)) as Material);

                // Colors in material need to be initialized with maximum length of the array (6 in this case)
                material.SetColorArray("_Colors", rainbowColors);
                material.SetInt("_ColorsNum", rainbowColors.Count);
            }

            OnValidate();
            mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        }

        public void OnEnable()
        {
            rglSubgraphVisualizationOutput.SetActive(visualizationOutputNodeId, true);
        }

        public void OnDisable()
        {
            rglSubgraphVisualizationOutput.SetActive(visualizationOutputNodeId, false);
        }

        public void OnValidate()
        {
            if (!material)
            {
                return;
            }

            material.SetFloat("_PointSize", pointSize);
            material.SetInt("_PointShape", (int)pointShape);

            if (!autoComputeColoringHeights) {
                material.SetFloat("_MinColoringHeight", minColoringHeight);
                material.SetFloat("_MaxColoringHeight", maxColoringHeight);
            }

            material.SetColorArray("_Colors", colors);
            material.SetInt("_ColorsNum", colors.Count);
        }

        public void SetPoints(Vector3[] points)
        {
            if (indices.Length < points.Length)
            {
                indices = new int[points.Length];
                for (int i = 0; i < points.Length; ++i)
                {
                    indices[i] = i;
                }
            }

            mesh.Clear();
            mesh.vertices = points;
            mesh.SetIndices(indices, 0, pointCount, MeshTopology.Points, 0);

            if (autoComputeColoringHeights)
            {
                minColoringHeight = mesh.bounds.min.y;
                maxColoringHeight = mesh.bounds.max.y;

                material.SetFloat("_MinColoringHeight", minColoringHeight);
                material.SetFloat("_MaxColoringHeight", maxColoringHeight);
            }
        }

        public void Update()
        {
            if (!sensor.enabled)
            {
                mesh.Clear();
            }
            Graphics.DrawMesh(mesh, Vector3.zero, Quaternion.identity, material, visualizationLayerID);
        }

        private void OnNewLidarData()
        {
            if (!enabled)
            {
                return;
            }
            pointCount = rglSubgraphVisualizationOutput.GetResultData<Vector3>(ref onlyHits);
            SetPoints(onlyHits);
        }
    }
}
