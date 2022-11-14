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
using System.Linq;
using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;

namespace RGLUnityPlugin
{
    /// <summary>
    /// Wrapper class for RGL graph concept.
    /// </summary>
    public class RGLGraph
    {
        private List<RGLNodeHandle> nodes = new List<RGLNodeHandle>();
        private List<RGLGraph> parentGraphs = new List<RGLGraph>();
        private List<RGLGraph> childGraphs = new List<RGLGraph>();
        private RGLField outputField = RGLField.UNKNOWN;

        //// MULTIGRAPH OPERATIONS ////
        public static void ConnectGraphs(RGLGraph parentGraph, RGLGraph childGraph)
        {
            if (parentGraph.nodes.Count == 0 || childGraph.nodes.Count == 0)
            {
                throw new RGLException("Attempted to connect empty graph!");
            }
            RGLNativeAPI.GraphNodeAddChild(parentGraph.nodes.Last().Node, childGraph.nodes.First().Node);
            parentGraph.childGraphs.Add(childGraph);
            childGraph.parentGraphs.Add(parentGraph);
        }

        public static void DisconnectGraphs(RGLGraph parentGraph, RGLGraph childGraph)
        {
            if (parentGraph.nodes.Count == 0 || childGraph.nodes.Count == 0)
            {
                throw new RGLException("Attempted to disconnect empty graph!");
            }

            if (!parentGraph.childGraphs.Contains(childGraph) || !childGraph.parentGraphs.Contains(parentGraph))
            {
                throw new RGLException("Attempted to disconnect graphs that are not connected!");
            }

            RGLNativeAPI.GraphNodeRemoveChild(parentGraph.nodes.Last().Node, childGraph.nodes.First().Node);
            parentGraph.childGraphs.Remove(childGraph);
            childGraph.parentGraphs.Remove(parentGraph);
        }

        //// ADD NODES ////
        public RGLGraph AddNodeRaysFromMat3x4f(string identifier, Matrix4x4[] rays)
        {
            ValidateNewNode(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodeRaysFromMat3x4f(ref handle.Node, rays);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.RAYS_FROM_MAT3X4F;
            AddNode(handle);
            return this;
        }

        public RGLGraph AddNodeRaysSetRingIds(string identifier, int[] ringIds)
        {
            ValidateNewNode(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodeRaysSetRingIds(ref handle.Node, ringIds);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.RAYS_SET_RING_IDS;
            AddNode(handle);
            return this;
        }

        public RGLGraph AddNodeRaysTransform(string identifier, Matrix4x4 transform)
        {
            ValidateNewNode(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodeRaysTransform(ref handle.Node, transform);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.RAYS_TRANSFORM;
            AddNode(handle);
            return this;
        }

        public RGLGraph AddNodeRaytrace(string identifier, float range)
        {
            ValidateNewNode(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodeRaytrace(ref handle.Node, range);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.RAYTRACE;
            AddNode(handle);
            return this;
        }

        public RGLGraph AddNodePointsTransform(string identifier, Matrix4x4 transform)
        {
            ValidateNewNode(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodePointsTransform(ref handle.Node, transform);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.POINTS_TRANSFORM;
            AddNode(handle);
            return this;
        }

        public RGLGraph AddNodePointsCompact(string identifier)
        {
            ValidateNewNode(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodePointsCompact(ref handle.Node);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.POINTS_COMPACT;
            AddNode(handle);
            return this;
        }

        public RGLGraph AddNodePointsDownsample(string identifier, Vector3 leafDims)
        {
            ValidateNewNode(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodePointsDownSample(ref handle.Node, leafDims);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.POINTS_DOWNSAMPLE;
            AddNode(handle);
            return this;
        }

        public RGLGraph AddNodePointsWritePCDFile(string identifier, string path)
        {
            ValidateNewNode(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodePointsWritePCDFile(ref handle.Node, path);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.POINTS_WRITE_PCD_FILE;
            AddNode(handle);
            return this;
        }

        public RGLGraph AddNodePointsFormat(string identifier, RGLField[] fields)
        {
            ValidateNewNode(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            if (fields.Length == 1)
            {
                RGLNativeAPI.NodePointsYield(ref handle.Node, fields);
                handle.Type = RGLNodeType.POINTS_YIELD;
                outputField = fields[0];
            }
            else
            {
                RGLNativeAPI.NodePointsFormat(ref handle.Node, fields);
                handle.Type = RGLNodeType.POINTS_FORMAT;
                outputField = RGLField.DYNAMIC_FORMAT;
            }
            handle.Identifier = identifier;
            AddNode(handle);
            return this;
        }

        //// UPDATE NODES ////
        public RGLGraph UpdateNodeRaysFromMat3x4f(string identifier, Matrix4x4[] rays)
        {
            ValidateExistingNode(identifier, RGLNodeType.RAYS_FROM_MAT3X4F);
            RGLNativeAPI.NodeRaysFromMat3x4f(ref nodes.Single(n => n.Identifier == identifier).Node, rays);
            return this;
        }

        public RGLGraph UpdateNodeRaysSetRingIds(string identifier, int[] ringIds)
        {
            ValidateExistingNode(identifier, RGLNodeType.RAYS_SET_RING_IDS);
            RGLNativeAPI.NodeRaysSetRingIds(ref nodes.Single(n => n.Identifier == identifier).Node, ringIds);
            return this;
        }

        public RGLGraph UpdateNodeRaysTransform(string identifier, Matrix4x4 transform)
        {
            ValidateExistingNode(identifier, RGLNodeType.RAYS_TRANSFORM);
            RGLNativeAPI.NodeRaysTransform(ref nodes.Single(n => n.Identifier == identifier).Node, transform);
            return this;
        }

        public RGLGraph UpdateNodeRaytrace(string identifier, float range)
        {
            ValidateExistingNode(identifier, RGLNodeType.RAYTRACE);
            RGLNativeAPI.NodeRaytrace(ref nodes.Single(n => n.Identifier == identifier).Node, range);
            return this;
        }

        public RGLGraph UpdateNodePointsTransform(string identifier, Matrix4x4 transform)
        {
            ValidateExistingNode(identifier, RGLNodeType.POINTS_TRANSFORM);
            RGLNativeAPI.NodePointsTransform(ref nodes.Single(n => n.Identifier == identifier).Node, transform);
            return this;
        }
        
        public RGLGraph UpdateNodePointsDownsample(string identifier, Vector3 leafDims)
        {
            ValidateExistingNode(identifier, RGLNodeType.POINTS_DOWNSAMPLE);
            RGLNativeAPI.NodePointsDownSample(ref nodes.Single(n => n.Identifier == identifier).Node, leafDims);
            return this;
        }

        public RGLGraph UpdateNodePointsWritePCDFile(string identifier, string path)
        {
            ValidateExistingNode(identifier, RGLNodeType.POINTS_WRITE_PCD_FILE);
            RGLNativeAPI.NodePointsWritePCDFile(ref nodes.Single(n => n.Identifier == identifier).Node, path);
            return this;
        }

        //// GRAPH OPERATIONS ////
        public int GetResultData<T>(ref T[] data) where T : unmanaged
        {
            ValidateOutputNode();
            unsafe
            {
                return RGLNativeAPI.GraphGetResult<T>(nodes.Last().Node, outputField, ref data, sizeof(T));
            }
        }

        public int GetResultDataRaw(ref byte[] data, int expectedPointSize)
        {
            ValidateOutputNode();
            return RGLNativeAPI.GraphGetResult<byte>(nodes.Last().Node, outputField, ref data, expectedPointSize);
        }

        public void Run()
        {
            if (nodes.Count == 0)
            {
                throw new RGLException("Attempted to run empty graph!");
            }
            RGLNativeAPI.GraphRun(nodes.First().Node);
        }

        public void Destroy()
        {
            DisconnectAllChildGraphs();
            DisconnectAllParentGraphs();
            if (nodes.Count > 0)
            {
                RGLNativeAPI.GraphDestroy(nodes.First().Node);
                nodes.Clear();
            }
        }

        //// PRIVATE HELPERS ////
        private void ValidateNewNode(string identifier)
        {
            var nodeFilter = nodes.Where(n => n.Identifier == identifier);
            if (nodeFilter.Count() != 0)
            {
                throw new RGLException($"Attempted to add node '{identifier}' twice!");
            }
        }

        private void ValidateExistingNode(string identifier, RGLNodeType desiredType)
        {
            var nodeFilter = nodes.Where(n => n.Identifier == identifier);
            if (nodeFilter.Count() != 1)
            {
                throw new RGLException($"Attempted to access node '{identifier}' but it was not found!");
            }
            if (nodeFilter.First().Type != desiredType)
            {
                throw new RGLException($"Attempted to access node '{identifier}' but node type mismatch!");
            }
        }

        private void ValidateOutputNode()
        {
            if (outputField == RGLField.UNKNOWN)
            {
                throw new RGLException("Attempted to get result data but format node was not found!");
            }
            RGLNodeType lastNodeType = nodes.Last().Type;
            if (!(lastNodeType == RGLNodeType.POINTS_FORMAT || lastNodeType == RGLNodeType.POINTS_YIELD))
            {
                throw new RGLException("Attempted to get result data but format node is not the last node in the graph!");
            }
        }

        private void AddNode(RGLNodeHandle nodeHandle)
        {
            if (nodes.Count == 0)
            {
                nodes.Add(nodeHandle);
                return;
            }

            if (childGraphs.Count > 0)
            {
                Debug.LogWarning("Added node to already connected graph. Removing child graphs...");
                DisconnectAllChildGraphs();
            }

            RGLNativeAPI.GraphNodeAddChild(nodes.Last().Node, nodeHandle.Node);
            nodes.Add(nodeHandle);
        }

        private void DisconnectAllChildGraphs()
        {
            foreach (RGLGraph graph in childGraphs.ToList())
            {
                DisconnectGraphs(this, graph);
            }
        }

        private void DisconnectAllParentGraphs()
        {
            foreach (RGLGraph graph in parentGraphs.ToList())
            {
                DisconnectGraphs(graph, this);
            }
        }

        ~RGLGraph()
        {
            Destroy();
        }
    }
}