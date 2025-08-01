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

using UnityEngine;
using Awsim.Common;
using System.Collections.Generic;
using System.Linq;

namespace Awsim.Usecase.PcdGeneration
{
    /// <summary>
    /// Provide functionality to visualize lanelet.
    /// </summary>
    [System.Serializable]
    public class LaneletVisualizer
    {
        [SerializeField]
        Material _material;

        [SerializeField]
        float _width;

        LaneletMap _laneletMap;

        public void Initialize(LaneletMap laneletMap)
        {
            _laneletMap = laneletMap;
        }

        public GameObject CreateCenterline(Transform parent)
        {
            var meshHolder = new GameObject("Lanelet");
            meshHolder.transform.parent = parent;
            var renderer = meshHolder.AddComponent<MeshRenderer>();
            var filter = meshHolder.AddComponent<MeshFilter>();
            filter.mesh = CreateMesh();
            renderer.material = _material;
            renderer.rayTracingMode = UnityEngine.Experimental.Rendering.RayTracingMode.Off;
            return meshHolder;
        }

        Mesh CreateMesh()
        {
            var mesh = new Mesh();
            var wayList = new List<Vector3[]>();
            var obj = new GameObject("Source", typeof(LineRenderer));
            var cameraHolder = new GameObject("Camera", typeof(Camera));
            var camera = cameraHolder.GetComponent<Camera>();
            camera.gameObject.tag = "Untagged";
            camera.transform.position = new Vector3(0, 10000, 0);
            camera.transform.forward = Vector3.down;
            obj.transform.forward = Vector3.up;
            mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
            var lineRenderer = obj.GetComponent<LineRenderer>();
            lineRenderer.alignment = LineAlignment.View;
            var laneletCount = _laneletMap.Lanelets.Count;
            var tmpMeshes = new Mesh[laneletCount];
            var combines = new CombineInstance[laneletCount];
            var lanelets = _laneletMap.Lanelets.Values.ToArray();
            for (int i = 0; i < laneletCount; ++i)
            {
                var lanelet = lanelets[i];
                tmpMeshes[i] = new Mesh();
                combines[i] = new CombineInstance
                {
                    transform = Matrix4x4.identity,
                    subMeshIndex = 0
                };
                var centerline = lanelet.CalculateCenterline();
                wayList.Add(centerline);
                lineRenderer.positionCount = centerline.Length;
                lineRenderer.SetPositions(centerline);
                lineRenderer.startWidth = _width;
                lineRenderer.endWidth = _width;
                lineRenderer.BakeMesh(tmpMeshes[i], camera);
                var colors = new Color[tmpMeshes[i].vertexCount];
                for (int v = 0; v < tmpMeshes[i].vertexCount; ++v)
                {
                    colors[v] = Color.yellow;
                }
                tmpMeshes[i].SetColors(colors);
                combines[i].mesh = tmpMeshes[i];
            }
            mesh.CombineMeshes(combines, true);
            for (int i = 0; i < tmpMeshes.Length; ++i)
            {
                Object.DestroyImmediate(tmpMeshes[i]);
            }
            Object.DestroyImmediate(cameraHolder);
            Object.DestroyImmediate(obj);
            return mesh;
        }
    }
}