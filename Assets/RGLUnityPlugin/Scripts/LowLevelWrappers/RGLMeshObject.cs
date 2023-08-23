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
// Experimental is necessary for gathering GraphicsFormat of the texture.
using UnityEngine.Experimental.Rendering;
using System.Collections.Generic;

namespace RGLUnityPlugin
{

    internal interface IRGLObject
    {
        GameObject RepresentedGO { get; }
        int? CategoryId { get; }
        string CategoryName { get; }
        
        void Update();

        void DestroyInRGL();
    }
    
    /// <summary>
    /// RGL counterpart of Unity GameObjects.
    /// Contains information about RGL entity and RGLMesh.
    /// </summary>
    public abstract class RGLObject<T> : IRGLObject
    {
        private readonly string identifier;
        private RGLMesh rglMesh;
        private RGLTexture rglTexture;
        
        public GameObject RepresentedGO { get; }
        public int? CategoryId { get; private set; }
        public string CategoryName { get; private set; }

        private IntPtr rglEntityPtr;

        protected abstract RGLMesh GetRGLMeshFrom(T meshSource);
        
        protected RGLObject(string identifier, GameObject representedGO, T meshSource)
        {
            this.identifier = identifier;
            RepresentedGO = representedGO;
            rglMesh = GetRGLMeshFrom(meshSource);
            if (rglMesh == null)
            {
                return;
            }
            UploadToRGL();
            SetIntensityTexture();

            var semanticCategory = RepresentedGO.GetComponentInParent<SemanticCategory>();
            if (semanticCategory != null)
            {
                semanticCategory.onCategoryIdChange += UpdateSemanticCategory;
                UpdateSemanticCategory(semanticCategory);
            }
        }

        ~RGLObject()
        {
            DestroyInRGL();
        }

        protected abstract void DestroyRGLMesh(RGLMesh rglMesh);
        
        public virtual void DestroyInRGL()
        {
            if (rglEntityPtr == IntPtr.Zero)
            {
                return;
            }
            
            RGLNativeAPI.CheckErr(RGLNativeAPI.rgl_entity_destroy(rglEntityPtr));
            rglEntityPtr = IntPtr.Zero;
            
            DestroyRGLMesh(rglMesh);
            rglMesh = null;
            
            if (rglTexture != null)
            {
                RGLTextureManager.UnregisterRGLTextureInstance(rglTexture);
                rglTexture = null;
            }
        }

        public void Update()
        {
            UpdateTransform();
            if (rglMesh is RGLSkinnedMesh rglSkinnedMesh)
            {
                rglSkinnedMesh.UpdateSkinnedMesh();
            }
        }

        protected abstract Matrix4x4 GetLocalToWorld();
        
        protected virtual void UpdateTransform()
        {
            if (rglEntityPtr == IntPtr.Zero)
            {
                return;
            }
            
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
            return identifier.GetHashCode();
        }

        public override bool Equals(object obj)
        {
            return obj is RGLObject<T> rglObject && identifier.Equals(rglObject.identifier);
        }

        private void UploadToRGL()
        {
            // Mesh should be uploaded.
            Assert.IsFalse(rglMesh.rglMeshPtr == IntPtr.Zero);

            unsafe
            {
                try
                {
                    RGLNativeAPI.CheckErr(
                        RGLNativeAPI.rgl_entity_create(out rglEntityPtr, IntPtr.Zero, rglMesh.rglMeshPtr));
                }
                catch (RGLException)
                {
                    if (rglEntityPtr != IntPtr.Zero) RGLNativeAPI.rgl_entity_destroy(rglEntityPtr);
                    throw;
                }
            }
        }

        private void UpdateSemanticCategory(SemanticCategory semanticCategory)
        {
            if (semanticCategory == null)
            {
                return;
            }

            CategoryId = semanticCategory.CategoryId;
            CategoryName = semanticCategory.gameObject.name;
            RGLNativeAPI.CheckErr(RGLNativeAPI.rgl_entity_set_id(rglEntityPtr, CategoryId.Value));
        }

        private void SetIntensityTexture()
        {
            var intensityTextureComponent = RepresentedGO.GetComponent<IntensityTexture>();

            if( intensityTextureComponent == null || intensityTextureComponent.texture == null)
            {
                return;
            }

            rglTexture = RGLTextureManager.RegisterRGLTextureInstance(intensityTextureComponent.texture);
            try
            {
                RGLNativeAPI.CheckErr(
                    RGLNativeAPI.rgl_entity_set_intensity_texture(rglEntityPtr, rglTexture.rglTexturePtr));
            }
            catch (RGLException)
            {
                Debug.LogError($"Cannot assign texture: {rglTexture.Identifier}, to entity: {identifier}");
                throw;
            }

            // Mesh should be uploaded before assigning UVs.
            Assert.IsFalse(rglMesh.rglMeshPtr == IntPtr.Zero);

            rglMesh.UploadUVs();
        }
    }

    public class RGLMeshObject : RGLObject<Mesh>
    {
        private readonly Func<Matrix4x4> getLocalToWorld;

        public RGLMeshObject(string identifier, GameObject representedGO, Mesh mesh, Func<Matrix4x4> getLocalToWorld) :
            base(identifier, representedGO, mesh)
        {
            this.getLocalToWorld = getLocalToWorld;
        }

        protected override RGLMesh GetRGLMeshFrom(Mesh mesh)
        {
            return RGLMeshManager.RegisterRGLMeshInstance(mesh);
        }

        protected override Matrix4x4 GetLocalToWorld()
        {
            return getLocalToWorld();
        }

        protected override void DestroyRGLMesh(RGLMesh rglMesh)
        {
            RGLMeshManager.UnregisterRGLMeshInstance(rglMesh);
        }
    }

    public class RGLMeshRendererObject : RGLObject<MeshRenderer>
    {
        private readonly Transform rendererTransform;

        public RGLMeshRendererObject(MeshRenderer meshRenderer) :
            base(
                $"{meshRenderer.gameObject.name}#{meshRenderer.gameObject.GetInstanceID()}",
                meshRenderer.gameObject,
                meshRenderer
            )
        {
            rendererTransform = meshRenderer.transform;
        }

        protected override RGLMesh GetRGLMeshFrom(MeshRenderer meshRenderer)
        {
            var meshFilter = meshRenderer.GetComponent<MeshFilter>();
            if (meshFilter.sharedMesh == null)
            {
                Debug.LogWarning($"Shared mesh of {meshRenderer.gameObject} is null, skipping");
                return null;
            }

            return RGLMeshManager.RegisterRGLMeshInstance(meshFilter.sharedMesh);
        }

        protected override Matrix4x4 GetLocalToWorld()
        {
            return rendererTransform.localToWorldMatrix;
        }

        protected override void DestroyRGLMesh(RGLMesh rglMesh)
        {
            RGLMeshManager.UnregisterRGLMeshInstance(rglMesh);
        }
    }

    public class RGLSkinnedMeshRendererObject : RGLObject<SkinnedMeshRenderer>
    {
        private readonly Transform skinnedMeshRendererTransform;

        public RGLSkinnedMeshRendererObject(SkinnedMeshRenderer skinnedMeshRenderer) :
            base(
                $"{skinnedMeshRenderer.gameObject.name}#{skinnedMeshRenderer.gameObject.GetInstanceID()}",
                skinnedMeshRenderer.gameObject,
                skinnedMeshRenderer
                )
        {
            skinnedMeshRendererTransform = skinnedMeshRenderer.transform;
        }

        protected override RGLMesh GetRGLMeshFrom(SkinnedMeshRenderer skinnedMeshRenderer)
        {
            return new RGLSkinnedMesh(skinnedMeshRenderer.gameObject.GetInstanceID(), skinnedMeshRenderer);
        }

        protected override Matrix4x4 GetLocalToWorld()
        {
            return skinnedMeshRendererTransform.localToWorldMatrix;
        }
        
        protected override void DestroyRGLMesh(RGLMesh rglMesh)
        {
            rglMesh.DestroyFromRGL();
        }
    }

    public class RGLColliderObject : RGLObject<Collider>
    {
        private readonly Collider collider;

        public RGLColliderObject(Collider collider) :
            base($"{collider.gameObject.name}#{collider.gameObject.GetInstanceID()}",
                collider.gameObject,
                collider
            )
        {
            this.collider = collider;
        }

        protected override RGLMesh GetRGLMeshFrom(Collider collider)
        {
            return RGLMeshManager.RegisterRGLMeshInstance(ColliderUtilities.GetMeshForCollider(collider));
        }

        protected override Matrix4x4 GetLocalToWorld()
        {
            return collider.transform.localToWorldMatrix * ColliderUtilities.GetColliderTransformMatrix(collider);
        }
        
        protected override void DestroyRGLMesh(RGLMesh rglMesh)
        {
            RGLMeshManager.UnregisterRGLMeshInstance(rglMesh);
        }
    }

    public class RGLTerrainObject : RGLObject<Terrain>
    {
        private readonly List<RGLMeshObject> terrainSubObjects;
        private readonly Transform terrainTransform;

        public RGLTerrainObject(Terrain terrain) :
            base($"{terrain.gameObject.name}#{terrain.gameObject.GetInstanceID()}", terrain.gameObject, terrain)
        {
            terrainTransform = terrain.transform;
            terrainSubObjects = new List<RGLMeshObject>();
            var terrainData = terrain.terrainData;
            for (var i = 0; i < terrainData.treeInstanceCount; i++)
            {
                var treeMesh = TerrainUtilities.GetTreeMesh(terrain, i);
                if (treeMesh is null)
                {
                    continue;
                }
                // we need to make a copy of the index because lambda captures use reference semantics for all captured variables
                var treeIndex = i;
                var tree = new RGLMeshObject($"{RepresentedGO.name}#{RepresentedGO.GetInstanceID()}#{treeIndex}",
                    RepresentedGO,
                    treeMesh,
                    () => terrain.transform.localToWorldMatrix * TerrainUtilities.GetTreePose(terrain, treeIndex));
                terrainSubObjects.Add(tree);
            }
        }
        
        protected override Matrix4x4 GetLocalToWorld()
        {
            return terrainTransform.localToWorldMatrix;
        }
        
        public override void DestroyInRGL()
        {
            foreach (var terrainSubObject in terrainSubObjects)
            {
                terrainSubObject.DestroyInRGL();
            }
            
            base.DestroyInRGL();
        }

        protected override void UpdateTransform()
        {
            foreach (var terrainSubObject in terrainSubObjects)
            {
                terrainSubObject.Update();
            }
            
            base.UpdateTransform();
        }

        protected override RGLMesh GetRGLMeshFrom(Terrain terrain)
        {
            return new RGLMesh(terrain.gameObject.GetInstanceID(), TerrainUtilities.GetTerrainMesh(terrain));
        }
        
        protected override void DestroyRGLMesh(RGLMesh rglMesh)
        {
            rglMesh.DestroyFromRGL();
        }
    }

    /// <summary>
    /// RGL counterpart of Unity Mesh.
    /// Contains information about RGL mesh.
    /// Can be constructed in multiple ways - e.g. from collider or a (non-skinned) mesh.
    /// </summary>
    public class RGLMesh
    {
        public int Identifier;
        public Mesh Mesh;

        public IntPtr rglMeshPtr = IntPtr.Zero;

        public RGLMesh(int identifier, Mesh mesh)
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

        public void UploadUVs()
        {
            Vector2[] UVs = Mesh.uv;
            bool uvOK = UVs != null && UVs.Length > 0;

            if(!uvOK)
            {
                Debug.LogWarning($"Could not assign UVs to mesh: {Identifier}. Mash has no UV, or UV are empty.");
            }
            else
            {
               unsafe
                {                    
                    fixed(Vector2* pUVs = UVs)
                    {
                        try
                        {
                            RGLNativeAPI.CheckErr(
                                RGLNativeAPI.rgl_mesh_set_texture_coords(rglMeshPtr,
                                (IntPtr)pUVs,
                                UVs.Length));
                        }
                        catch (RGLException)
                        {
                            Debug.LogWarning($"Could not assign UVs to mesh: {Identifier}.");
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

        public RGLSkinnedMesh(int identifier, SkinnedMeshRenderer smr)
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

    /// <summary>
    /// RGL counterpart of Unity Texture.
    /// Contains information about RGL texture.
    /// </summary>
    public class RGLTexture
    {
        public int Identifier;
        public Texture2D Texture;
        public IntPtr rglTexturePtr = IntPtr.Zero;

        public RGLTexture(){}
        
        public RGLTexture(Texture2D texture, int identifier)
        {
            Identifier = identifier;
            Texture = texture;
            UploadToRGL();
        }

        ~RGLTexture()
        {
            DestroyFromRGL();
        }

        public void DestroyFromRGL()
        {
            if (rglTexturePtr != IntPtr.Zero)
            {
                RGLNativeAPI.CheckErr(RGLNativeAPI.rgl_texture_destroy(rglTexturePtr));
                rglTexturePtr = IntPtr.Zero;                
            }
        }

        protected void UploadToRGL()
        {
            bool resolutionOK = Texture.width > 0 && Texture.height > 0;
            bool graphicsFormatOK = Texture.graphicsFormat == GraphicsFormat.R8_UNorm;

            if (!resolutionOK)
            {
                throw new NotSupportedException(
                    $"Could not get texture data. Resolution seems to be broken.");
            }
            
            if (!graphicsFormatOK)
            {
                throw new NotSupportedException(
                    $"Could not get texture data. Texture format has to be equal to R8_UNorm.");
            }
           
            unsafe
            {
                fixed (void* textureDataPtr = Texture.GetRawTextureData())
                {
                    try
                    {     
                        RGLNativeAPI.CheckErr(
                            RGLNativeAPI.rgl_texture_create(
                            out rglTexturePtr,
                            (IntPtr) textureDataPtr,
                            Texture.width,
                            Texture.height));
                    }
                    catch (RGLException)
                    {
                        if (rglTexturePtr != IntPtr.Zero) 
                        {
                            RGLNativeAPI.rgl_texture_destroy(rglTexturePtr);
                            rglTexturePtr = IntPtr.Zero;                            
                        }      
                        throw;                      
                    }
               }  
            }       
        }
    }
}