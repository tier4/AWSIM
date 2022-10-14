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
using PclSharp;
using Unity.Collections;
using UnityEngine;

namespace RGLUnityPlugin
{
    /// <summary>
    /// This class provides automatic creation & destruction of the native Lidar object.
    /// </summary>
    public class RGLGraph
    {
        private RGLNodeHandle fromMat3x4fRayNode = new RGLNodeHandle(RGLNodeType.RAYS_FROM_MAT3X4F);
        private RGLNodeHandle setRingIdsRayNode = new RGLNodeHandle(RGLNodeType.RAYS_SET_RING_IDS);
        private RGLNodeHandle transformRaysNode = new RGLNodeHandle(RGLNodeType.RAYS_TRANSFORM);
        private RGLNodeHandle raytraceNode = new RGLNodeHandle(RGLNodeType.RAYTRACE);
        private RGLNodeHandle compactNode = new RGLNodeHandle(RGLNodeType.POINTS_COMPACT);
        private RGLNodeHandle transformPointsToLidarFrameNode = new RGLNodeHandle(RGLNodeType.POINTS_TRANSFORM);

        public RGLGraph()
        {
            SetRays(new Matrix4x4[1] {Matrix4x4.identity}); // Create FromMat3x4fRaysNode
            SetRingIds(new int[1] {0}); // Create SetRingIdsRaysNode
            SetLidarPosition(Matrix4x4.identity); // Create transformRaysNode
            SetLidarRange(Mathf.Infinity); // Create raytraceNode
            RGLNativeAPI.NodePointsCompact(ref compactNode.node);
            RGLNativeAPI.NodePointsTransform(ref transformPointsToLidarFrameNode.node, Matrix4x4.identity);

            //                                                 -> [many children, see AddFormat]
            //                                                /
            // UseRays -> TransformRays -> Raytrace -> Compact -> TransformPointsToLidarFrame -> [many children]
            RGLNativeAPI.GraphNodeAddChild(fromMat3x4fRayNode.node, setRingIdsRayNode.node);
            RGLNativeAPI.GraphNodeAddChild(setRingIdsRayNode.node, transformRaysNode.node);
            RGLNativeAPI.GraphNodeAddChild(transformRaysNode.node, raytraceNode.node);
            RGLNativeAPI.GraphNodeAddChild(raytraceNode.node, compactNode.node);
            RGLNativeAPI.GraphNodeAddChild(compactNode.node, transformPointsToLidarFrameNode.node);
        }

        public RGLNodeHandle GetPointsWorldFrameNodeHandle()
        {
            return compactNode;
        }

        public RGLNodeHandle GetPointsLidarFrameNodeHandle()
        {
            return transformPointsToLidarFrameNode;
        }

        public void SetRays(Matrix4x4[] rays)
        {
            RGLNativeAPI.NodeRaysFromMat3x4f(ref fromMat3x4fRayNode.node,  rays);
        }

        public void SetRingIds(int[] ringIds)
        {
            RGLNativeAPI.NodeRaysSetRingIds(ref setRingIdsRayNode.node, ringIds);
        }

        public void SetLidarPosition(Matrix4x4 localToWorld)
        {
            RGLNativeAPI.NodeRaysTransform(ref transformRaysNode.node, localToWorld);
            RGLNativeAPI.NodePointsTransform(ref transformPointsToLidarFrameNode.node, localToWorld.inverse);
        }

        public void SetLidarRange(float range)
        {
            RGLNativeAPI.NodeRaytrace(ref raytraceNode.node, range);
        }

        public RGLNodeHandle AddPointsTransform(RGLNodeHandle parentNode, Matrix4x4 transform)
        {
            RGLNodeHandle handle = new RGLNodeHandle(RGLNodeType.POINTS_TRANSFORM);
            RGLNativeAPI.NodePointsTransform(ref handle.node, transform);
            RGLNativeAPI.GraphNodeAddChild(parentNode.node, handle.node);
            return handle;
        }

        public void UpdatePointsTransform(RGLNodeHandle nodeHandle, Matrix4x4 transform)
        {
            if (nodeHandle.GetNodeType() != RGLNodeType.POINTS_TRANSFORM)
            {
                throw new RGLException($"Node type mismatch, requested update for {RGLNodeType.POINTS_TRANSFORM}, but got {nodeHandle.GetNodeType()}");
            }
            RGLNativeAPI.NodePointsTransform(ref nodeHandle.node, transform);
        }

        public RGLOutputHandle AddFormat(RGLNodeHandle parentNode, RGLField[] fields)
        {
            RGLOutputHandle leaf = new RGLOutputHandle();
            if (fields.Length == 1)
            {
                RGLNativeAPI.NodePointsYield(ref leaf.node, fields);
                leaf.field = fields[0];
            }
            else
            {
                RGLNativeAPI.NodePointsFormat(ref leaf.node, fields);
                leaf.field = RGLField.DYNAMIC_FORMAT;
            }
            RGLNativeAPI.GraphNodeAddChild(parentNode.node, leaf.node);
            return leaf;
        }

        public void Run()
        {
            RGLNativeAPI.GraphRun(raytraceNode.node);
        }

        public int GetData<T>(RGLOutputHandle handle, ref T[] data) where T : unmanaged
        {
            unsafe
            {
                return RGLNativeAPI.GraphGetResult<T>(handle.node, handle.field, ref data, sizeof(T));
            }
        }

        public int GetDataRaw(RGLOutputHandle handle, ref byte[] data, int expectedPointSize)
        {
            return RGLNativeAPI.GraphGetResult<byte>(handle.node, handle.field, ref data, expectedPointSize);
        }

        public void SetGaussianNoiseParamsCtx(LidarNoiseParams param)
        {
            // TODO
            Debug.LogError("Gaussian Noise not yet supported");
        }

        ~RGLGraph()
        {
            RGLNativeAPI.GraphDestroy(raytraceNode.node);
        }
    }
}