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

using System.Collections.Generic;
using System.Linq;
using UnityEngine.Serialization;
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

        static private readonly List<Color> rainblowColors = new List<Color> {
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
        [Range(0, 0.5f)]private float pointSize = 0.05f;

        [SerializeField]
        private List<Color> colors = rainblowColors;

        [SerializeField]
        private bool autoComputeColoringHeights = false;

        [SerializeField]
        private float minColoringHeight = 0f;

        [SerializeField]
        private float maxColoringHeight = 10f;

        private Material material = null;

        private static readonly int visualizationLayerID = 11;

        private Mesh mesh;

        public void Start()
        {
            mesh = new Mesh();
            if (!material)
            {
                material = Instantiate<Material>(Resources.Load("PointCloudMaterial", typeof(Material)) as Material);

                // Colors in material need to be initialized with maximum length of the array (6 in this case)
                material.SetColorArray("_Colors", rainblowColors);
                material.SetInt("_ColorsNum", rainblowColors.Count);
            }

            OnValidate();
            mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
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
            if (autoComputeColoringHeights)
            {
                material.SetFloat("_MinColoringHeight", points.Min(p => p.y));
                material.SetFloat("_MaxColoringHeight", points.Max(p => p.y));
            }

            // TODO: easy, low-prio optimization here
            int[] indicies = new int[points.Length];
            Color[] colors = new Color[points.Length];

            for (int i = 0; i < points.Length; ++i)
            {
                indicies[i] = i;
                colors[i] = Color.red; // Can base on reflectivity or whatever here
            }

            mesh.Clear();
            mesh.vertices = points;
            mesh.colors = colors;
            mesh.SetIndices(indicies, MeshTopology.Points, 0);
        }

        public void Update()
        {
            Graphics.DrawMesh(mesh, Vector3.zero, Quaternion.identity, material, visualizationLayerID);
        }
    }
}