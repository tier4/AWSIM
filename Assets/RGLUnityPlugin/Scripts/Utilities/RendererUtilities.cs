using System.Collections.Generic;
using UnityEngine;

namespace RGLUnityPlugin
{
    public class RendererUtilities
    {
        /// <summary>
        /// Some prefabs have LOD Group component (Level Of Detail)
        /// which implies using different renders based on the distance from the camera.
        /// For raytracing purposes, the best available LOD is used (index: 0).
        /// Some prefabs may not use LOD and use mesh renderers directly.
        /// This function takes the aforementioned into account to output exactly one Renderer per game object.
        /// It is guaranteed that yielded meshes are enabled and are not attached to a disabled LOD.
        /// </summary>
        public static IEnumerable<Renderer> GetUniqueRenderersInGameObjects(IEnumerable<GameObject> gameObjects)
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

                bool hasMeshRenderer = gameObject.TryGetComponent<MeshRenderer>(out var mr) && mr.enabled;
                bool hasMeshFilter = gameObject.TryGetComponent<MeshFilter>(out _); // Mesh filter can't be disabled
                if (hasMeshRenderer && hasMeshFilter)
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
        
        public static List<GameObject> GetAllGameObjectsOfPrefab(GameObject prefab)
        {
            var result = new List<GameObject>();
            GetAllGameObjectsOfPrefabHelper(prefab.transform, result);
            return result;
        }
        
        private static void GetAllGameObjectsOfPrefabHelper(Transform transform, List<GameObject> result)
        {
            result.Add(transform.gameObject);
            for (var i = 0; i < transform.childCount; i++)
            {
                GetAllGameObjectsOfPrefabHelper(transform.GetChild(i), result);
            }
        }
    }
}