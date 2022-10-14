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

using UnityEngine;

namespace RGLUnityPlugin
{
    [System.Serializable]
    public class PointCloudVisualization : MonoBehaviour
    {
        public Material material;
        private static readonly int visualizationLayerID = 11;

        private Mesh mesh;

        public void Start()
        {
            mesh = new Mesh();
            if (!material)
            {
                material = Resources.Load("PointCloudMaterial", typeof(Material)) as Material;
            }

            mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        }

        public void SetPoints(Vector3[] points)
        {
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