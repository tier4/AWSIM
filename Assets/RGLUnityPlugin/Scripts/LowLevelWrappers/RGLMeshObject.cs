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
using UnityEngine;
using UnityEngine.Assertions;

namespace RGLUnityPlugin
{
    /// <summary>
    /// RGL counterpart of Unity GameObjects.
    /// Contains information about RGL entity and RGLMesh.
    /// </summary>
    public class RGLObject
    {
        public string Identifier;
        public RGLMesh RglMesh;
        public Func<Matrix4x4> GetLocalToWorld;
        public GameObject RepresentedGO;

        public IntPtr rglEntityPtr;

        public RGLObject(string identifier, RGLMesh rglMesh, Func<Matrix4x4> getLocalToWorld, GameObject representedGO)
        {
            Identifier = identifier;
            RglMesh = rglMesh;
            GetLocalToWorld = getLocalToWorld;
            RepresentedGO = representedGO;

            UploadToRGL();
        }

        ~RGLObject()
        {
            DestroyFromRGL();
        }

        public void DestroyFromRGL()
        {
            if (rglEntityPtr != IntPtr.Zero)
            {
                RGLNativeAPI.CheckErr(RGLNativeAPI.rgl_entity_destroy(rglEntityPtr));
                rglEntityPtr = IntPtr.Zero;
            }
        }

        public void UpdateTransform()
        {
            Matrix4x4 m = GetLocalToWorld();
            float[] matrix3x4 =
            {
                m.m00, m.m01, m.m02, m.m03,
                m.m10, m.m11, m.m12, m.m13,
                m.m20, m.m21, m.m22, m.m23,
            };
            unsafe
            {
                fixed (float* pMatrix3x4 = matrix3x4)
                {
                    RGLNativeAPI.CheckErr(
                        RGLNativeAPI.rgl_entity_set_pose(rglEntityPtr, (IntPtr) pMatrix3x4));
                }
            }
        }

        public override int GetHashCode()
        {
            return Identifier.GetHashCode();
        }

        public override bool Equals(object obj)
        {
            return obj is RGLObject rglObject && Identifier.Equals(rglObject.Identifier);
        }

        protected void UploadToRGL()
        {
            // Mesh should be uploaded.
            Assert.IsFalse(RglMesh.rglMeshPtr == IntPtr.Zero);

            unsafe
            {
                try
                {
                    RGLNativeAPI.CheckErr(
                        RGLNativeAPI.rgl_entity_create(out rglEntityPtr, IntPtr.Zero, RglMesh.rglMeshPtr));
                }
                catch (RGLException)
                {
                    if (rglEntityPtr != IntPtr.Zero) RGLNativeAPI.rgl_entity_destroy(rglEntityPtr);
                    throw;
                }
            }
        }
    }

    /// <summary>
    /// RGL counterpart of Unity Mesh.
    /// Contains information about RGL mesh.
    /// Can be constructed in multiple ways - e.g. from collider or a (non-skinned) mesh.
    /// </summary>
    public class RGLMesh
    {
        public string Identifier;
        public Mesh Mesh;

        public IntPtr rglMeshPtr = IntPtr.Zero;

        public RGLMesh(string identifier, Mesh mesh)
        {
            Identifier = identifier;
            Mesh = mesh;

            UploadToRGL();
        }

        ~RGLMesh()
        {
            DestroyFromRGL();
        }

        public void DestroyFromRGL()
        {
            if (rglMeshPtr != IntPtr.Zero)
            {
                RGLNativeAPI.CheckErr(RGLNativeAPI.rgl_mesh_destroy(rglMeshPtr));
                rglMeshPtr = IntPtr.Zero;
            }
        }

        protected RGLMesh() {}

        protected void UploadToRGL()
        {
            Vector3[] vertices = Mesh.vertices;
            int[] indices = Mesh.triangles;
            bool verticesOK = vertices != null && vertices.Length > 0;
            bool indicesOK = indices != null && indices.Length > 0;

            if (!verticesOK || !indicesOK)
            {
                throw new NotSupportedException(
                    $"Could not get mesh data with mesh identifier {Identifier}. The mesh may be not readable or empty.");
            }

            unsafe
            {
                fixed (Vector3* pVertices = vertices)
                {
                    fixed (int* pIndices = indices)
                    {
                        try
                        {
                            RGLNativeAPI.CheckErr(
                                RGLNativeAPI.rgl_mesh_create(out rglMeshPtr,
                                    (IntPtr) pVertices, vertices.Length,
                                    (IntPtr) pIndices, indices.Length / 3));
                        }
                        catch (RGLException)
                        {
                            if (rglMeshPtr != IntPtr.Zero) RGLNativeAPI.rgl_mesh_destroy(rglMeshPtr);
                            throw;
                        }
                    }
                }
            }
        }
    }

    /// <summary>
    /// Some objects (such as NPC) use skinned meshes, which needs to be constantly updated by the Unity side.
    /// </summary>
    public class RGLSkinnedMesh : RGLMesh
    {
        public SkinnedMeshRenderer SkinnedMeshRenderer;

        public RGLSkinnedMesh(string identifier, SkinnedMeshRenderer smr)
        {
            Identifier = identifier;
            Mesh = new Mesh();
            SkinnedMeshRenderer = smr;
            SkinnedMeshRenderer.BakeMesh(Mesh, true);
            UploadToRGL();
        }

        public void UpdateSkinnedMesh()
        {
            SkinnedMeshRenderer.BakeMesh(Mesh, true);
            unsafe
            {
                // Accessing .vertices perform a CPU copy!
                // TODO: This could be optimized using Vulkan-CUDA interop and Unity NativePluginInterface. Expect difficulties.
                // https://docs.unity3d.com/ScriptReference/Mesh.GetNativeVertexBufferPtr.html
                // https://docs.unity3d.com/Manual/NativePluginInterface.html
                // https://github.com/NVIDIA/cuda-samples/tree/master/Samples/5_Domain_Specific/simpleVulkan
                fixed (Vector3* pVertices = Mesh.vertices)
                {
                    RGLNativeAPI.CheckErr(
                        RGLNativeAPI.rgl_mesh_update_vertices(rglMeshPtr, (IntPtr) pVertices, Mesh.vertices.Length));
                }
            }
        }
    }
}