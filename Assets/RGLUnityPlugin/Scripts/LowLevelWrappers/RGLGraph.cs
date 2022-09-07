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
using UnityEngine;

namespace RGLUnityPlugin
{
    /// <summary>
    /// This class provides automatic creation & destruction of the native Lidar object.
    /// </summary>
    public class RGLGraph
    {
        // private IntPtr gra;
        private IntPtr useRaysNode = IntPtr.Zero;
        private IntPtr useRingIdsNode = IntPtr.Zero;
        private IntPtr transformRays = IntPtr.Zero;
        private IntPtr raytraceNode = IntPtr.Zero;
        private IntPtr compactNode = IntPtr.Zero;

        public RGLGraph()
        {
            SetRays(new Matrix4x4[1] {Matrix4x4.identity}); // Create useRaysNode
            SetRingIds(new int[1] {0});
            SetLidarPosition(Matrix4x4.identity); // Create transformRaysNode
            SetLidarRange(Mathf.Infinity); // Create raytraceNode
            RGLNativeAPI.NodeCompact(ref compactNode);

            // UseRays -> TransformRays -> Raytrace -> Compact -> [many children, see AddFormat]
            RGLNativeAPI.GraphNodeAddChild(useRaysNode, transformRays);
            RGLNativeAPI.GraphNodeAddChild(transformRays, raytraceNode);
            RGLNativeAPI.GraphNodeAddChild(raytraceNode, compactNode);
        }

        public void SetRays(Matrix4x4[] rays)
        {
            RGLNativeAPI.NodeUseMat3x4f(ref useRaysNode,  rays);
        }

        public void SetLidarPosition(Matrix4x4 localToWorld)
        {
            RGLNativeAPI.NodeTransformRays(ref transformRays, Matrix4x4.identity);
        }

        public void SetLidarRange(float range)
        {
            RGLNativeAPI.NodeRaytrace(ref raytraceNode, range);
        }

        public RGLOutputHandle AddFormat(RGLField[] fields, Matrix4x4 postRaycastTransform)
        {
            IntPtr parentOfLeafNode = compactNode;
            if (!postRaycastTransform.isIdentity)
            {
                // This leak is conscious, RGL is able to cleanup correctly whole graph provided only a single node.
                IntPtr transformPointsNode = IntPtr.Zero;
                RGLNativeAPI.NodeTransformPoints(ref transformPointsNode, postRaycastTransform);
                RGLNativeAPI.GraphNodeAddChild(compactNode, transformPointsNode);
                parentOfLeafNode = transformPointsNode;

            }

            RGLOutputHandle leaf = new RGLOutputHandle();
            if (fields.Length == 1)
            {
                RGLNativeAPI.NodeYieldPoints(ref leaf.node, fields);
                leaf.field = fields[0];
            }
            else
            {
                RGLNativeAPI.NodeFormat(ref leaf.node, out leaf.field, fields);
            }
            RGLNativeAPI.GraphNodeAddChild(parentOfLeafNode, leaf.node);
            return leaf;
        }

        public void SetRingIds(int[] ringIds)
        {
            RGLNativeAPI.NodeUseRingIds(ref useRingIdsNode, ringIds);
        }


        public int GetData<T>(RGLOutputHandle handle, ref T[] data) where T : unmanaged
        {
            unsafe
            {
                return RGLNativeAPI.GraphGetResult(handle.node, handle.field, ref data, sizeof(T));
            }
        }

        public int GetDataRaw(RGLOutputHandle handle, ref byte[] data, int expectedPointSize)
        {
            return RGLNativeAPI.GraphGetResult(handle.node, handle.field, ref data, expectedPointSize);
        }

        public void SetGaussianNoiseParamsCtx(LidarNoiseParams param)
        {
            // TODO
            Debug.LogError("Gaussian Noise not yet supported");
        }

        ~RGLGraph()
        {
            RGLNativeAPI.GraphDestroy(raytraceNode);
        }
    }
}