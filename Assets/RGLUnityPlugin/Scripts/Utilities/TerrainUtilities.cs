using System.Collections.Generic;
using UnityEngine.Rendering;
using UnityEngine;

namespace RGLUnityPlugin
{
    public class TerrainUtilities
    {
        public static Mesh GetTerrainMesh(Terrain terrain)
        {
            var terrainData = terrain.terrainData;
            var heightmapResolution = terrainData.heightmapResolution;
            var holesResolution = terrainData.holesResolution;
            var correctHolesResolution = holesResolution == heightmapResolution - 1;
            var heights = terrainData.GetHeights(0, 0, heightmapResolution, heightmapResolution);
            var solidSurfaceTiles = terrainData.GetHoles(0, 0, holesResolution, holesResolution);
            var scale = terrainData.heightmapScale;
            var vertices = new Vector3[heightmapResolution * heightmapResolution];
            
            if (!correctHolesResolution)
            {
                Debug.LogWarning($"Terrain {terrain.GetInstanceID()} holes resolution is incorrect, holes will be ignored by RGL");
            }

            for (var z = 0; z < heightmapResolution; z++)
            {
                for (var x = 0; x < heightmapResolution; x++)
                {
                    vertices[x * heightmapResolution + z].x =  x * scale.x;
                    vertices[x * heightmapResolution + z].y = heights[z, x] * scale.y;
                    vertices[x * heightmapResolution + z].z =  z * scale.z;
                }
            }

            // this is the number of squares alongside each axis of the terrain
            // e.g. if you have a 3x3 grid of points you can fill the space between them using a 2x2 square grid
            var tileResolution = heightmapResolution - 1;
            
            
            // count solid terrain tiles (not holes)
            var tileCount = 0;
            foreach (var solidSurface in solidSurfaceTiles)
            {
                if (solidSurface)
                {
                    tileCount++;
                }
            }

            if (!correctHolesResolution)
            {
                tileCount = tileResolution * tileResolution;
            }
            
            // there are 2 triangles per square tile, so 6 indices
            var triangles = new int[tileCount * 2 * 3];

            var trianglesIndex = 0;
            for (var z = 0; z < tileResolution; z++)
            {
                for (var x = 0; x < tileResolution; x++)
                {
                    
                    if (correctHolesResolution && !solidSurfaceTiles[z, x])
                    {
                        continue;
                    }
                    
                    var sampleBase = x * heightmapResolution + z;
                    
                    // first triangle of tile
                    triangles[trianglesIndex++] = sampleBase;
                    triangles[trianglesIndex++] = sampleBase + heightmapResolution;
                    triangles[trianglesIndex++] = sampleBase + heightmapResolution + 1;
                    
                    // second triangle of tile
                    triangles[trianglesIndex++] = sampleBase;
                    triangles[trianglesIndex++] = sampleBase + 1;
                    triangles[trianglesIndex++] = sampleBase + 1 + heightmapResolution;
                }
            }
            
            var uv = new Vector2[vertices.Length];
            for (var i = 0; i < vertices.Length; i++)
            {
                uv[i] = new Vector2(vertices[i].x / (tileResolution * scale.x), vertices[i].z / (tileResolution * scale.z));
            }
            
            var heightmapMesh = new Mesh();
            heightmapMesh.indexFormat = IndexFormat.UInt32;
            heightmapMesh.vertices = vertices;
            heightmapMesh.triangles = triangles;
            heightmapMesh.uv = uv;

            return heightmapMesh;
        }

        // public static Mesh GetTreeMesh(Terrain terrain, int treeIndex)
        // {
        //     var terrainData = terrain.terrainData;
        //     
        //     if (treePrefab.TryGetComponent(out LODGroup lodGroup))
        //     {
        //         if (lodGroup.GetLODs()[0].renderers[0].TryGetComponent(out MeshFilter meshFilter))
        //         {
        //             return meshFilter.sharedMesh;
        //         }
        //
        //         Debug.LogWarning($"Tree[{treeIndex}] \"{treePrefab.name}\" of terrain {terrain.GetInstanceID()} has LODGroup component with no MeshFilter component, it will be ignored by RGL");
        //         return null;
        //     }
        //     else if (treePrefab.TryGetComponent(out MeshFilter meshFilter))
        //     {
        //         Debug.LogWarning($"Tree[{treeIndex}] \"{treePrefab.name}\" of terrain {terrain.GetInstanceID()} has no LODGroup component, but it has a MeshFilter component. " +
        //                          $"The tree's rotation and scale is not going o match those in the Unity Editor, " +
        //                          $"since Unity Editor has a bug and does not rotate or scale trees without LODGroup components");
        //         return meshFilter.sharedMesh;
        //     }
        //     Debug.LogWarning($"Tree[{treeIndex}] \"{treePrefab.name}\" of terrain {terrain.GetInstanceID()} has no LODGroup or MeshFilter component, it will be ignored by RGL");
        //     return null;
        // }

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
