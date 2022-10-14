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
using System.Linq;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.Profiling;

namespace RGLUnityPlugin
{
    /// <summary>
    /// This object is meant to be a singleton responsible for synchronizing model data between Unity and RGL.
    /// On every frame, it detects changes happening on the scene and reacts appropriately.
    /// It allows to use three different strategies to obtain 3D model for game objects.
    /// Strategies are implemented as private methods IntoRGLObjects* and called via IntoRGLObject delegate.
    ///
    /// Scene Manager will ignore:
    /// - Disabled Game Objects
    /// - Implicitly disabled game objects (e.g. disabled parent, grandparent, etc.)
    /// - Disabled colliders
    /// - Disabled MeshRenderers and SkinnedMeshRenderers
    /// - MRs and SMRs that belong to a disabled LOD
    /// </summary>
    public class SceneManager : MonoBehaviour
    {
        public enum MeshSource
        {
            OnlyColliders,
            RegularMeshesAndCollidersInsteadOfSkinned,
            RegularMeshesAndSkinnedMeshes
        }

        [SerializeField]
        private MeshSource meshSource = MeshSource.RegularMeshesAndSkinnedMeshes;

        // Getting meshes strategies
        private delegate IEnumerable<RGLObject> IntoRGLObjectsStrategy(IEnumerable<GameObject> gameObjects);

        private IntoRGLObjectsStrategy IntoRGLObjects;

        // Keeping track of the scene objects
        private HashSet<GameObject> lastFrameGameObjects = new HashSet<GameObject>();
        private readonly Dictionary<GameObject, RGLObject> uploadedRGLObjects = new Dictionary<GameObject, RGLObject>();

        private int lastUpdateFrame = -1;

        void OnDisable()
        {
            Clear();
        }

        void OnValidate()
        {
            UpdateMeshSource();
        }

        private void Awake()
        {
            RGLNativeAPI.CheckVersion();
            if (IntoRGLObjects == null)
            {
                UpdateMeshSource();
            }
        }

        private void UpdateMeshSource()
        {
            Clear();
            IntoRGLObjects = meshSource switch
            {
                MeshSource.OnlyColliders => IntoRGLObjectsUsingCollider,
                MeshSource.RegularMeshesAndCollidersInsteadOfSkinned => IntoRGLObjectsHybrid,
                MeshSource.RegularMeshesAndSkinnedMeshes => IntoRGLObjectsUsingMeshes,
                _ => throw new ArgumentOutOfRangeException()
            };
            Debug.Log($"RGL mesh source: {meshSource.ToString()}");
        }

        /// <summary>
        /// Find out what changes happened on the scene since the last update and update RGL data.
        /// </summary>
        public void DoUpdate()
        {
            /* TODO(prybicki):
             * Placing this code in Update() might cause an artifact - only undefined subset of
             * spawned / despawned game objects would be visible. (due to the Update()s ordering)
             * A better place would be EarlyUpdate, which would introduce a 1 frame delay,
             * but guarantee consistency (exclude dependency on the Update() ordering)
             * However, EarlyUpdate does not exist, but could be implemented via PlayerLoop.
             * It would be also allow to hide async computations related to RGL - skinning and raytracing.
             */

            // The following snippet is a consequence of inability to easily synchronize with LiDAR publishing.
            // TODO: rework it in the future
            if (lastUpdateFrame == Time.frameCount)
            {
                return; // Already done in this frame 
            }

            lastUpdateFrame = Time.frameCount;


            Profiler.BeginSample("Find changes and make TODO list");
            var thisFrameGOs = FindObjectsOfType<GameObject>().Where(go => go.activeInHierarchy);

            // Added
            var toAddGOs = new HashSet<GameObject>(thisFrameGOs);
            toAddGOs.ExceptWith(lastFrameGameObjects);
            RGLObject[] toAdd = IntoRGLObjects(toAddGOs)
                .Where(o => o.RepresentedGO.GetComponentsInParent<LidarSensor>().Length == 0).ToArray();

            // Removed
            var toRemoveGOs = new HashSet<GameObject>(lastFrameGameObjects);
            toRemoveGOs.ExceptWith(thisFrameGOs);
            toRemoveGOs.IntersectWith(uploadedRGLObjects.Keys);
            RGLObject[] toRemove = toRemoveGOs.Select((o) => uploadedRGLObjects[o]).ToArray();

            // Skinned
            RGLObject[] newToSkin = toAdd.Where(o => o is RGLSkinnedObject).ToArray();
            RGLObject[] existingToSkin =
                uploadedRGLObjects.Values.Where(o => o is RGLSkinnedObject).Except(toRemove).ToArray();
            RGLObject[] toSkin = existingToSkin.Concat(newToSkin).ToArray();
            
            lastFrameGameObjects = new HashSet<GameObject>(thisFrameGOs);
            Profiler.EndSample();

            Profiler.BeginSample("Remove despawned objects");
            foreach (var rglObject in toRemove)
            {
                RemoveObject(rglObject);
            }

            Profiler.EndSample();

            // TODO: this can be parallelized and moved to Update()
            Profiler.BeginSample("Perform CPU skinning");
            foreach (var rglObject in toSkin)
            {
                var skinnedObject = rglObject as RGLSkinnedObject;
                Assert.IsNotNull(skinnedObject);
                skinnedObject.UpdateSkinnedMesh();
            }

            Profiler.EndSample();

            Profiler.BeginSample("Refresh skinned models on the GPU");
            foreach (var rglObject in existingToSkin)
            {
                UpdateObjectVertices(rglObject);
            }

            Profiler.EndSample();

            Profiler.BeginSample("Add spawned objects");
            foreach (var rglObject in toAdd)
            {
                UploadObject(rglObject);
            }

            Profiler.EndSample();


            // TODO(prybicki): This part can take up to 8ms on Shinjuku scene, two ideas to optimize it soon:
            // - Implement batch update in RGL
            // - Use Transform.hasChanged to filter out some objects
            Profiler.BeginSample("Update transforms");
            foreach (var gameRGLObject in uploadedRGLObjects)
            {
                UpdateObjectTransform(gameRGLObject.Value);
            }

            Profiler.EndSample();
        }

        private void Clear()
        {
            if (!lastFrameGameObjects.Any() && !uploadedRGLObjects.Any()) return;
            while (uploadedRGLObjects.Any())
            {
                RemoveObject(uploadedRGLObjects.First().Value);
            }

            uploadedRGLObjects.Clear();
            lastFrameGameObjects.Clear();
            Debug.Log("RGLSceneManager: cleared");
        }

        /// <summary>
        /// Yields a collection of RGL objects based on active colliders found in provided game objects.
        /// This function ignores whether gameObject is active, if filtering is needed, it should be done earlier.
        /// </summary>
        private static IEnumerable<RGLObject> IntoRGLObjectsUsingCollider(IEnumerable<GameObject> gameObjects)
        {
            foreach (var gameObject in gameObjects)
            {
                if (gameObject.TryGetComponent<Collider>(out var collider))
                {
                    if (!collider.enabled) continue;
                    if (collider.GetType() == typeof(WheelCollider))
                    {
                        // WheelCollider, unlike other types of Colliders, does not have 3D mesh representation.
                        // https://docs.unity3d.com/Manual/class-WheelCollider.html
                        // It is basically a 2D ring, which is used to compute interactions with the ground.
                        // I.e. it does not include any information on tire width, so it's difficult to generate any mesh.
                        // At the moment, WheelColliders are used only in the Lexus Ego vehicle,
                        // therefore, as long as E2E does not support multiple Egos, it won't be a problem.
                        // In the future, collider-based lidar may be dropped at all,
                        // or an additional (inactive) mesh collider may be added along with WheelCollider.
                        continue;
                    }

                    yield return ColliderToRGLObject(collider);
                }
            }
        }

        /// <summary>
        /// Yields a collection of RGL objects found in provided game objects, based on
        /// - Mesh for non-skinned MeshRenderers
        /// - Set of colliders (if provided) attached to the rootBone and below for SkinnedMeshRenderers.
        /// This function ignores whether gameObject is active, if filtering is needed, it should be done earlier.
        /// </summary>
        private static IEnumerable<RGLObject> IntoRGLObjectsHybrid(IEnumerable<GameObject> gameObjects)
        {
            var collidersToYield = new HashSet<Collider>();
            foreach (var renderer in GetUniqueRenderersInGameObjects(gameObjects))
            {
                if (renderer is SkinnedMeshRenderer smr)
                {
                    if (smr.rootBone == null)
                    {
                        Debug.LogWarning($"Root bone of {smr.gameObject} is null (hence no colliders), skipping");
                        continue;
                    }
                    // Some SkinnedMeshRenderers may point to a common root bone.
                    var boneColliders = smr.rootBone.gameObject.GetComponentsInChildren<Collider>();
                    var enabledBoneColliders = boneColliders.Where(c => c.enabled);
                    collidersToYield.UnionWith(enabledBoneColliders);
                }

                if (renderer is MeshRenderer mr)
                {
                    var mf = mr.GetComponent<MeshFilter>();
                    yield return MeshFilterToRGLObject(mf);
                }
            }

            foreach (var collider in collidersToYield)
            {
                yield return ColliderToRGLObject(collider);
            }
        }

        /// <summary>
        /// Yields a collection of RGL objects found in provided game objects,
        /// based on MeshRenderer and SkinnedMeshRenderer.
        /// This function ignores whether gameObject is active, if filtering is needed, it should be done earlier.
        /// </summary>
        private static IEnumerable<RGLObject> IntoRGLObjectsUsingMeshes(IEnumerable<GameObject> gameObjects)
        {
            foreach (var renderer in GetUniqueRenderersInGameObjects(gameObjects))
            {
                var go = renderer.gameObject;
                if (renderer is MeshRenderer mr)
                {
                    var mf = mr.GetComponent<MeshFilter>();
                    yield return MeshFilterToRGLObject(mf);
                }

                if (renderer is SkinnedMeshRenderer smr)
                {
                    yield return new RGLSkinnedObject
                    {
                        GetLocalToWorld = () => smr.transform.localToWorldMatrix,
                        Identifier = $"{go.name}#{go.GetInstanceID()}",
                        SkinnedMeshRenderer = smr,
                        Mesh = new Mesh(), // Actual mesh will be generated (skinned) later
                        RepresentedGO = go
                    };
                }
            }
        }

        private static RGLObject ColliderToRGLObject(Collider collider)
        {
            return new RGLObject
            {
                GetLocalToWorld = () => collider.transform.localToWorldMatrix *
                                        ColliderUtilities.GetColliderTransformMatrix(collider),
                Mesh = ColliderUtilities.GetMeshForCollider(collider),
                Identifier = $"{collider.gameObject.name}#{collider.gameObject.GetInstanceID()}",
                RepresentedGO = collider.gameObject
            };
        }

        private static RGLObject MeshFilterToRGLObject(MeshFilter meshFilter)
        {
            var gameObject = meshFilter.gameObject;
            return new RGLObject
            {
                GetLocalToWorld = () => gameObject.transform.localToWorldMatrix,
                Identifier = $"{gameObject.name}#{gameObject.GetInstanceID()}",
                Mesh = meshFilter.mesh,
                RepresentedGO = meshFilter.gameObject
            };
        }


        /// <summary>
        /// Some prefabs have LOD Group component (Level Of Detail)
        /// which implies using different renders based on the distance from the camera.
        /// For raytracing purposes, the best available LOD is used (index: 0).
        /// Some prefabs may not use LOD and use mesh renderers directly.
        /// This function takes the aforementioned into account to output exactly one Renderer per game object.
        /// It is guaranteed that yielded meshes are enabled and are not attached to a disabled LOD.
        /// </summary>
        private static IEnumerable<Renderer> GetUniqueRenderersInGameObjects(IEnumerable<GameObject> gameObjects)
        {
            var lodGroups = new HashSet<LODGroup>();
            var smrs = new HashSet<SkinnedMeshRenderer>();
            var mrs = new HashSet<MeshRenderer>();

            foreach (var gameObject in gameObjects)
            {
                if (gameObject.TryGetComponent<LODGroup>(out var lodGroup))
                {
                    lodGroups.Add(lodGroup);
                }

                if (gameObject.TryGetComponent<SkinnedMeshRenderer>(out var smr) && smr.enabled)
                {
                    smrs.Add(smr);
                }
                
                if (gameObject.TryGetComponent<MeshRenderer>(out var mr) && mr.enabled)
                {
                    mrs.Add(mr);
                }
            }

            foreach (var lodGroup in lodGroups)
            {
                for (int lodIndex = 0; lodIndex < lodGroup.lodCount; ++lodIndex)
                {
                    var lod = lodGroup.GetLODs()[lodIndex];
                    foreach (var renderer in lod.renderers)
                    {
                        // In theory it is possible that the renderer-containing game object is not a descendant of the LoD
                        // therefore we need to additionally check if renderer.gameObject.activeInHierarchy
                        if (lodGroup.enabled && lodIndex == 0 && renderer.enabled && renderer.gameObject.activeInHierarchy)
                        {
                            yield return renderer;
                        }

                        smrs.Remove(renderer as SkinnedMeshRenderer);
                        mrs.Remove(renderer as MeshRenderer);
                    }
                }
            }

            foreach (var smr in smrs) yield return smr;
            foreach (var mr in mrs) yield return mr;
        }

        private void UploadObject(RGLObject rglObject)
        {
            // Game Objects must not have duplicate representations.
            Assert.IsFalse(uploadedRGLObjects.ContainsKey(rglObject.RepresentedGO));

            Vector3[] vertices = rglObject.Mesh.vertices;
            int[] indices = rglObject.Mesh.triangles;
            bool verticesOK = vertices != null && vertices.Length > 0;
            bool indicesOK = indices != null && indices.Length > 0;

            if (!verticesOK || !indicesOK)
            {
                throw new NotSupportedException(
                    $"Could not get mesh data of {rglObject.Identifier}. The mesh may be not readable or empty.");
            }

            // TODO: Mesh & Entity could be separated here
            unsafe
            {
                fixed (Vector3* pVertices = vertices)
                {
                    fixed (int* pIndices = indices)
                    {
                        try
                        {
                            RGLNativeAPI.CheckErr(
                                RGLNativeAPI.rgl_mesh_create(out rglObject.rglMesh,
                                    (IntPtr) pVertices, vertices.Length,
                                    (IntPtr) pIndices, indices.Length / 3));
                            RGLNativeAPI.CheckErr(
                                RGLNativeAPI.rgl_entity_create(out rglObject.rglEntity, IntPtr.Zero,
                                    rglObject.rglMesh));
                        }
                        catch (RGLException)
                        {
                            if (rglObject.rglMesh != IntPtr.Zero) RGLNativeAPI.rgl_mesh_destroy(rglObject.rglMesh);
                            throw;
                        }
                    }
                }
            }

            uploadedRGLObjects.Add(rglObject.RepresentedGO, rglObject);
        }

        private void RemoveObject(RGLObject rglObject)
        {
            if (rglObject.rglEntity != IntPtr.Zero)
            {
                RGLNativeAPI.CheckErr(RGLNativeAPI.rgl_entity_destroy(rglObject.rglEntity));
                rglObject.rglEntity = IntPtr.Zero;
            }

            if (rglObject.rglMesh != IntPtr.Zero)
            {
                RGLNativeAPI.CheckErr(RGLNativeAPI.rgl_mesh_destroy(rglObject.rglMesh));
                rglObject.rglMesh = IntPtr.Zero;
            }

            uploadedRGLObjects.Remove(rglObject.RepresentedGO);
        }

        private static void UpdateObjectTransform(RGLObject rglObject)
        {
            Matrix4x4 m = rglObject.GetLocalToWorld();
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
                        RGLNativeAPI.rgl_entity_set_pose(rglObject.rglEntity, (IntPtr) pMatrix3x4));
                }
            }
        }

        private void UpdateObjectVertices(RGLObject rglObject)
        {
            Assert.IsTrue(uploadedRGLObjects.ContainsKey(rglObject.RepresentedGO));
            unsafe
            {
                // Accessing .vertices perform a CPU copy!
                // TODO: This could be optimized using Vulkan-CUDA interop and Unity NativePluginInterface. Expect difficulties.
                // https://docs.unity3d.com/ScriptReference/Mesh.GetNativeVertexBufferPtr.html
                // https://docs.unity3d.com/Manual/NativePluginInterface.html
                // https://github.com/NVIDIA/cuda-samples/tree/master/Samples/5_Domain_Specific/simpleVulkan
                fixed (Vector3* pVertices = rglObject.Mesh.vertices)
                {
                    RGLNativeAPI.CheckErr(
                        RGLNativeAPI.rgl_mesh_set_vertices(rglObject.rglMesh, (IntPtr) pVertices,
                            rglObject.Mesh.vertices.Length));
                }
            }
        }
    }
}
