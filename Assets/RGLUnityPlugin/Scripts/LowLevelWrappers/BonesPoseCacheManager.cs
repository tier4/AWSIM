// Copyright 2024 Robotec.ai.
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
using UnityEngine;

namespace RGLUnityPlugin
{
    /// <summary>
    /// Since multiple skinned meshes can share the same skeleton, BonesPoseCacheManager improves the performance of the simulation
    /// by caching the calculated pose of the skeletons for the given simulation step.
    /// </summary>
    public static class BonesPoseCacheManager
    {
        private class BonesPose
        {
            public BonesPose(SkinnedMeshRenderer smr)
            {
                this.smr = smr;
                var bonesCount = smr.bones.Length;
                pose = new Matrix4x4[bonesCount];
                rglPose = new float[bonesCount * 3 * 4]; // Mat3x4 == 12 floats
                usageCount = 1;
            }

            public SkinnedMeshRenderer smr; // needed to retrieve the current skeleton's pose
            public Matrix4x4[] pose; // buffer for the skeleton's pose in the Unity representation
            public float[] rglPose; // buffer for the skeleton's pose in the RGL representation
            public int usageCount; // the counter on how many meshes use this skeleton
            public int lastUpdateFrame = -1; // simulation frame
            public int lastFixedUpdateFrame = -1; // physics cycle in the simulation frame
        }

        // The bone root acts as the identifier of the skeleton.
        private static Dictionary<Transform, BonesPose> boneRootToBonesPose = new Dictionary<Transform, BonesPose>();

        public static void RegisterBonesPoseInstance(SkinnedMeshRenderer smr)
        {
            if (!boneRootToBonesPose.ContainsKey(smr.rootBone))
            {
                boneRootToBonesPose.Add(smr.rootBone, new BonesPose(smr));
            }
            else
            {
                boneRootToBonesPose[smr.rootBone].usageCount++;
            }
        }

        public static void UnregisterBonesPoseInstance(Transform rootBone)
        {
            if (!boneRootToBonesPose.ContainsKey(rootBone))
            {
                Debug.LogWarning(
                    $"Trying to unregister absent in BonesPoseCacheManager rootBone: '{rootBone.name}', ignoring request");
                return;
            }

            var bonesPose = boneRootToBonesPose[rootBone];
            bonesPose.usageCount--;
            if (bonesPose.usageCount == 0)
            {
                boneRootToBonesPose.Remove(rootBone);
            }
        }

        public static float[] GetRglPose(Transform rootBone)
        {
            if (!boneRootToBonesPose.ContainsKey(rootBone))
            {
                throw new NotSupportedException(
                    $"Trying to get RglPose from root bone ('{rootBone.name}') that is not registered in BonesPoseCacheManager");
            }

            var bonesPose = boneRootToBonesPose[rootBone];

            if (SceneManager.Instance.LastUpdateFrame == bonesPose.lastUpdateFrame &&
                SceneManager.Instance.LastFixedUpdateFrame == bonesPose.lastFixedUpdateFrame)
            {
                return bonesPose.rglPose;
            }

            var bones = bonesPose.smr.bones;
            for (int i = 0; i < bonesPose.pose.Length; i++)
            {
                bonesPose.pose[i] = bones[i].localToWorldMatrix;
            }

            RGLNativeAPI.IntoMat3x4f(bonesPose.pose, ref bonesPose.rglPose);

            bonesPose.lastUpdateFrame = SceneManager.Instance.LastUpdateFrame;
            bonesPose.lastFixedUpdateFrame = SceneManager.Instance.LastFixedUpdateFrame;
            return bonesPose.rglPose;
        }

        public static void Clear()
        {
            boneRootToBonesPose.Clear();
        }
    }
}
