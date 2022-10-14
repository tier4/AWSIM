using AWSIM.Lanelet;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace AWSIM.PointCloudMapping
{
    /// <summary>
    /// Provide functionality to visualize lanelet.
    /// </summary>
    [System.Serializable]
    public class LaneletVisualizer
    {
        [SerializeField]
        private Material material;
        
        [SerializeField]
        private float width;

        private LaneletMap laneletMap;

        public void Initialize(LaneletMap laneletMap)
        {
            this.laneletMap = laneletMap;
        }

        public GameObject CreateCenterline(Transform parent)
        {
            var meshHolder = new GameObject("Lanelet");
            meshHolder.transform.parent = parent;
            var renderer = meshHolder.AddComponent<MeshRenderer>();
            var filter = meshHolder.AddComponent<MeshFilter>();
            filter.mesh = CreateMesh();
            renderer.material = material;
            renderer.rayTracingMode = UnityEngine.Experimental.Rendering.RayTracingMode.Off;
            return meshHolder;
        }

        private Mesh CreateMesh()
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
            var laneletCount = laneletMap.Lanelets.Count;
            var tmpMeshes = new Mesh[laneletCount];
            var combines = new CombineInstance[laneletCount];
            var lanelets = laneletMap.Lanelets.Values.ToArray();
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
                lineRenderer.startWidth = width;
                lineRenderer.endWidth = width;
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
