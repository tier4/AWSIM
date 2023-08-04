
using UnityEngine.Rendering;
using UnityEngine;

namespace RGLUnityPlugin
{
    public class TerrainUtilities
    {
        public static Mesh GetTerrainMesh(Terrain terrain)
        {
            var terrainData = terrain.terrainData;
            var resolution = terrainData.heightmapResolution;
            var heights = terrainData.GetHeights(0, 0, resolution, resolution);
            var scale = terrainData.heightmapScale;
            var vertices = new Vector3[resolution * resolution];

            for (var x = 0; x < resolution; x++)
            {
                for (var z = 0; z < resolution; z++)
                {
                    vertices[x * resolution + z].x =  x * scale.x;
                    vertices[x * resolution + z].y = heights[z, x] * scale.y;
                    vertices[x * resolution + z].z =  z * scale.z;
                }
            }

            var squareResolution = resolution - 1;
            // there are 2 triangles per square, so 6 indices
            var triangles = new int[squareResolution * squareResolution * 2 * 3];
            
            for (var x = 0; x < squareResolution; x++)
            {
                for (var z = 0; z < squareResolution; z++)
                {
                    var sampleBase = x * resolution + z;
                    var squareBase = 6 * (x * squareResolution + z);
                    
                    // first triangle of square
                    triangles[squareBase] = sampleBase;
                    triangles[squareBase + 1] = sampleBase + resolution;
                    triangles[squareBase + 2] = sampleBase + resolution + 1;
                    
                    // second triangle of square
                    triangles[squareBase + 3] = sampleBase;
                    triangles[squareBase + 4] = sampleBase + 1;
                    triangles[squareBase + 5] = sampleBase + 1 + resolution;
                }
            }

            var heightmapMesh = new Mesh();
            heightmapMesh.indexFormat = IndexFormat.UInt32;
            heightmapMesh.vertices = vertices;
            heightmapMesh.triangles = triangles;

            return heightmapMesh;
        }

        public static Mesh GetTreeMesh(Terrain terrain, int treeIndex)
        {
            var terrainData = terrain.terrainData;
            var treeInstance = terrainData.GetTreeInstance(treeIndex);
            var treePrefab = terrainData.treePrototypes[treeInstance.prototypeIndex].prefab;
            if (treePrefab.TryGetComponent(out LODGroup lodGroup))
            {
                if (lodGroup.GetLODs()[0].renderers[0].TryGetComponent(out MeshFilter meshFilter))
                {
                    return meshFilter.sharedMesh;
                }

                Debug.LogWarning($"Tree[{treeIndex}] of terrain {terrain}'s LODGroup component has no MeshFilter component, it will be ignored by LIDAR");
                return null;
            }
            Debug.LogWarning($"Tree[{treeIndex}] of terrain {terrain} has no LODGroup component, it will be ignored by LIDAR");
            return null;
        }

        public static Matrix4x4 GetTreePose(Terrain terrain, int treeIndex)
        {
            var terrainPosition = terrain.transform.position;
            var terrainData = terrain.terrainData;
            var resolution = terrainData.heightmapResolution;
            var heightmapScale = terrainData.heightmapScale;
            var treeInstance = terrainData.treeInstances[treeIndex];
            var treePosition = treeInstance.position;

            var translation = new Vector3(treePosition.x, 0, treePosition.z);
            translation.x *= heightmapScale.x * (resolution - 1);
            translation.z *= heightmapScale.z * (resolution - 1);
            var samplePose = new Vector3(terrainPosition.x + translation.x, 0, terrainPosition.z + translation.z);
            translation.y = terrain.SampleHeight(samplePose);

            var rotation = Quaternion.AngleAxis(treeInstance.rotation * Mathf.Rad2Deg, Vector3.up);

            var scale = new Vector3(treeInstance.widthScale, treeInstance.heightScale, treeInstance.widthScale);

            return Matrix4x4.TRS(translation, rotation, scale);
        }
    }
}
