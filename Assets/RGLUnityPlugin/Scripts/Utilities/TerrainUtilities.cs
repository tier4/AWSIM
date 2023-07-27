
using UnityEngine.Rendering;
using UnityEngine;
using System;

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

            for (var i = 0; i < resolution; i++)
            {
                for (var j = 0; j < resolution; j++)
                {
                    vertices[i * resolution + j].x =  i * scale.x;
                    vertices[i * resolution + j].y = heights[j, i] * scale.y;
                    vertices[i * resolution + j].z =  j * scale.z;
                }
            }

            var quadResolution = resolution - 1;
            var triangles = new int[quadResolution * quadResolution * 2 * 3];
            
            for (var i = 0; i < quadResolution; i++)
            {
                for (var j = 0; j < quadResolution; j++)
                {
                    var sampleBase = i * resolution + j;
                    var quadBase = 6 * (i * quadResolution + j);
                    
                    // first triangle of quad
                    triangles[quadBase] = sampleBase;
                    triangles[quadBase + 1] = sampleBase + resolution;
                    triangles[quadBase + 2] = sampleBase + resolution + 1;
                    
                    // second triangle of quad
                    triangles[quadBase + 3] = sampleBase;
                    triangles[quadBase + 4] = sampleBase + 1;
                    triangles[quadBase + 5] = sampleBase + resolution + 1;
                }
            }

            var mesh = new Mesh();
            mesh.indexFormat = IndexFormat.UInt32;
            mesh.vertices = vertices;
            mesh.triangles = triangles;

            return mesh;
        }

        public static Mesh GetTreeMesh(Terrain terrain, int treeIndex)
        {
            var terrainData = terrain.terrainData;
            var treeInstance = terrainData.treeInstances[treeIndex];
            var treePrototype = terrainData.treePrototypes[treeInstance.prototypeIndex];
            return treePrototype.prefab.GetComponent<MeshFilter>().sharedMesh;
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

            var rotation = Quaternion.AngleAxis((float)(treeInstance.rotation / Math.PI * 180), Vector3.up);

            var scale = new Vector3(treeInstance.widthScale, treeInstance.heightScale, treeInstance.widthScale);

            return Matrix4x4.TRS(translation, rotation, scale);
        }
    }
}