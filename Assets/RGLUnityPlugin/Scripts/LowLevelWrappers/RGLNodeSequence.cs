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
    /// Represents a linearly connected series of RGL nodes.
    /// </summary>
    public class RGLNodeSequence
    {
        private List<RGLNodeHandle> nodes = new List<RGLNodeHandle>();
        private List<RGLNodeSequence> parents = new List<RGLNodeSequence>();
        private List<RGLNodeSequence> childs = new List<RGLNodeSequence>();
        private RGLField outputField = RGLField.UNKNOWN;
        private bool isActive = true; 

        //// NODESEQUENCES OPERATIONS ////
        public static void Connect(RGLNodeSequence parent, RGLNodeSequence child)
        {
            if (parent.nodes.Count == 0 || child.nodes.Count == 0)
            {
                throw new RGLException("Attempted to connect empty NodeSequence!");
            }

            if (parent.childs.Contains(child) || child.parents.Contains(parent))
            {
                throw new RGLException("Attempted to connect NodeSequences twice!");
            }

            RGLNativeAPI.GraphNodeAddChild(parent.nodes.Last().Node, child.nodes.First().Node);
            parent.childs.Add(child);
            child.parents.Add(parent);
        }

        public static void Disconnect(RGLNodeSequence parent, RGLNodeSequence child)
        {
            if (parent.nodes.Count == 0 || child.nodes.Count == 0)
            {
                throw new RGLException("Attempted to disconnect empty NodeSequence!");
            }

            if (!parent.childs.Contains(child) || !child.parents.Contains(parent))
            {
                throw new RGLException("Attempted to disconnect NodeSequences that are not connected!");
            }

            RGLNativeAPI.GraphNodeRemoveChild(parent.nodes.Last().Node, child.nodes.First().Node);
            parent.childs.Remove(child);
            child.parents.Remove(parent);
        }

        //// ADD NODES ////
        public RGLNodeSequence AddNodeRaysFromMat3x4f(string identifier, Matrix4x4[] rays)
        {
            ValidateNewNode(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodeRaysFromMat3x4f(ref handle.Node, rays);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.RAYS_FROM_MAT3X4F;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodeRaysSetRingIds(string identifier, int[] ringIds)
        {
            ValidateNewNode(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodeRaysSetRingIds(ref handle.Node, ringIds);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.RAYS_SET_RING_IDS;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodeRaysTransform(string identifier, Matrix4x4 transform)
        {
            ValidateNewNode(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodeRaysTransform(ref handle.Node, transform);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.RAYS_TRANSFORM;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodeRaytrace(string identifier, float range)
        {
            ValidateNewNode(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodeRaytrace(ref handle.Node, range);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.RAYTRACE;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodePointsTransform(string identifier, Matrix4x4 transform)
        {
            ValidateNewNode(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodePointsTransform(ref handle.Node, transform);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.POINTS_TRANSFORM;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodePointsCompact(string identifier)
        {
            ValidateNewNode(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodePointsCompact(ref handle.Node);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.POINTS_COMPACT;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodePointsDownsample(string identifier, Vector3 leafDims)
        {
            ValidateNewNode(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodePointsDownSample(ref handle.Node, leafDims);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.POINTS_DOWNSAMPLE;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodePointsWritePCDFile(string identifier, string path)
        {
            ValidateNewNode(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodePointsWritePCDFile(ref handle.Node, path);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.POINTS_WRITE_PCD_FILE;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodePointsYield(string identifier, RGLField field)
        {
            ValidateNewNode(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodePointsYield(ref handle.Node, new [] {field});
            handle.Type = RGLNodeType.POINTS_YIELD;
            outputField = field;
            handle.Identifier = identifier;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodePointsFormat(string identifier, RGLField[] fields)
        {
            ValidateNewNode(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodePointsFormat(ref handle.Node, fields);
            handle.Type = RGLNodeType.POINTS_FORMAT;
            outputField = RGLField.DYNAMIC_FORMAT;
            handle.Identifier = identifier;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodePointsRos2Publish(
            string identifier, string topicName, string frameId,
            RGLQosPolicyReliability reliability = RGLQosPolicyReliability.QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            RGLQosPolicyDurability durability = RGLQosPolicyDurability.QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,
            RGLQosPolicyHistory history = RGLQosPolicyHistory.QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
            int historyDepth = 5)
        {
            ValidateNewNode(identifier);
            if (nodes.Count == 0 || nodes.Last().Type != RGLNodeType.POINTS_FORMAT)
            {
                throw new RGLException("Attempted to add NodePointsRos2Publish but NodePointsFormat is required before!");
            }
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodePointsRos2PublishWithQos(ref handle.Node, topicName, frameId, reliability, durability, history, historyDepth);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.POINTS_ROS2_PUBLISH;
            AddNode(handle);
            return this;
        }

        //// UPDATE NODES ////
        public RGLNodeSequence UpdateNodeRaysFromMat3x4f(string identifier, Matrix4x4[] rays)
        {
            ValidateExistingNode(identifier, RGLNodeType.RAYS_FROM_MAT3X4F);
            RGLNativeAPI.NodeRaysFromMat3x4f(ref nodes.Single(n => n.Identifier == identifier).Node, rays);
            return this;
        }

        public RGLNodeSequence UpdateNodeRaysSetRingIds(string identifier, int[] ringIds)
        {
            ValidateExistingNode(identifier, RGLNodeType.RAYS_SET_RING_IDS);
            RGLNativeAPI.NodeRaysSetRingIds(ref nodes.Single(n => n.Identifier == identifier).Node, ringIds);
            return this;
        }

        public RGLNodeSequence UpdateNodeRaysTransform(string identifier, Matrix4x4 transform)
        {
            ValidateExistingNode(identifier, RGLNodeType.RAYS_TRANSFORM);
            RGLNativeAPI.NodeRaysTransform(ref nodes.Single(n => n.Identifier == identifier).Node, transform);
            return this;
        }

        public RGLNodeSequence UpdateNodeRaytrace(string identifier, float range)
        {
            ValidateExistingNode(identifier, RGLNodeType.RAYTRACE);
            RGLNativeAPI.NodeRaytrace(ref nodes.Single(n => n.Identifier == identifier).Node, range);
            return this;
        }

        public RGLNodeSequence UpdateNodePointsTransform(string identifier, Matrix4x4 transform)
        {
            ValidateExistingNode(identifier, RGLNodeType.POINTS_TRANSFORM);
            RGLNativeAPI.NodePointsTransform(ref nodes.Single(n => n.Identifier == identifier).Node, transform);
            return this;
        }
        
        public RGLNodeSequence UpdateNodePointsDownsample(string identifier, Vector3 leafDims)
        {
            ValidateExistingNode(identifier, RGLNodeType.POINTS_DOWNSAMPLE);
            RGLNativeAPI.NodePointsDownSample(ref nodes.Single(n => n.Identifier == identifier).Node, leafDims);
            return this;
        }

        public RGLNodeSequence UpdateNodePointsWritePCDFile(string identifier, string path)
        {
            ValidateExistingNode(identifier, RGLNodeType.POINTS_WRITE_PCD_FILE);
            RGLNativeAPI.NodePointsWritePCDFile(ref nodes.Single(n => n.Identifier == identifier).Node, path);
            return this;
        }

        //// NODESEQUENCE OPERATIONS ////
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
                throw new RGLException("Attempted to run empty NodeSequence!");
            }
            RGLNativeAPI.GraphRun(nodes.First().Node);
        }

        public void RemoveNode(string identifier)
        {
            int index = nodes.FindIndex(n => n.Identifier == identifier);

            if (index == -1)
            {
                throw new RGLException($"Attempted to remove node '{identifier}' but it was not found!");
            }

            if (nodes.Count == 1)
            {
                Debug.LogWarning("Removed the last node in the NodeSequence. Disconnecting parents & childs...");
                Clear();
                return;
            }

            // If removing output node then unset outputField
            RGLNodeType nodeType = nodes[index].Type;
            if (nodeType == RGLNodeType.POINTS_FORMAT || nodeType == RGLNodeType.POINTS_YIELD)
            {
                outputField = RGLField.UNKNOWN;
            }

            // Removing first node in NodeSequence
            if (index == 0)
            {
                if (parents.Count > 0)
                {
                    Debug.LogWarning("Removed the first node in already connected NodeSequence. Disconnecting parents...");
                    DisconnectAllParents();
                }
                RGLNativeAPI.GraphNodeRemoveChild(nodes[0].Node, nodes[1].Node);
                RGLNativeAPI.GraphDestroy(nodes[index].Node);
                nodes.RemoveAt(index);
                // Need to inactive a new first node if NodeSequence was inactive.
                if (!isActive)
                {
                    isActive = true;
                    SetActive(false);
                }
                return;
            }

            // Removing last node in NodeSequence
            if (index == nodes.Count - 1)
            {
                if (childs.Count > 0)
                {
                    Debug.LogWarning("Removed the last node in already connected NodeSequence. Disconnecting childs...");
                    DisconnectAllChilds();
                }
                RGLNativeAPI.GraphNodeRemoveChild(nodes[index - 1].Node, nodes[index].Node);
                RGLNativeAPI.GraphDestroy(nodes[index].Node);
                nodes.RemoveAt(index);
                return;
            }

            // Removing node in the middle of NodeSequence
            RGLNativeAPI.GraphNodeRemoveChild(nodes[index - 1].Node, nodes[index].Node);
            RGLNativeAPI.GraphNodeRemoveChild(nodes[index].Node, nodes[index + 1].Node);
            RGLNativeAPI.GraphNodeAddChild(nodes[index - 1].Node, nodes[index + 1].Node);
            RGLNativeAPI.GraphDestroy(nodes[index].Node);
            nodes.RemoveAt(index);
        }

        public void Clear()
        {
            DisconnectAllChilds();
            DisconnectAllParents();
            if (nodes.Count > 0)
            {
                RGLNativeAPI.GraphDestroy(nodes.First().Node);
                nodes.Clear();
            }
            isActive = true;
        }

        public bool IsActive()
        {
            return isActive;
        }

        public void SetActive(bool active)
        {
            if (isActive == active)
            {
                return;
            }

            if (nodes.Count == 0)
            {
                throw new RGLException("Attempted to set active on empty NodeSequence!");
            }

            RGLNativeAPI.GraphNodeSetActive(nodes.First().Node, active);
            isActive = active;
        }

        //// PRIVATE HELPERS ////
        // A new node to add should not exists in our sequence
        private void ValidateNewNode(string identifier)
        {
            var nodeFilter = nodes.Where(n => n.Identifier == identifier);
            if (nodeFilter.Count() != 0)
            {
                throw new RGLException($"Attempted to add node '{identifier}' twice!");
            }
        }

        // A desired node should exists in our sequence and has the same type as requested
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

        // To get result data specific node should be at the end of sequence
        private void ValidateOutputNode()
        {
            if (outputField == RGLField.UNKNOWN)
            {
                throw new RGLException("Attempted to get result data but format or yield node was not found!");
            }
            RGLNodeType lastNodeType = nodes.Last().Type;
            if (!(lastNodeType == RGLNodeType.POINTS_FORMAT || lastNodeType == RGLNodeType.POINTS_YIELD))
            {
                throw new RGLException("Attempted to get result data but format or yield node is not the last node in the NodeSequence!");
            }
        }

        private void AddNode(RGLNodeHandle nodeHandle)
        {
            if (nodes.Count == 0)
            {
                nodes.Add(nodeHandle);
                return;
            }

            if (childs.Count > 0)
            {
                throw new RGLException("Attempted to add node to already connected NodeSequence!");
            }

            RGLNativeAPI.GraphNodeAddChild(nodes.Last().Node, nodeHandle.Node);
            nodes.Add(nodeHandle);
        }

        private void DisconnectAllChilds()
        {
            foreach (RGLNodeSequence nodeSequence in childs.ToList())
            {
                Disconnect(this, nodeSequence);
            }
        }

        private void DisconnectAllParents()
        {
            foreach (RGLNodeSequence nodeSequence in parents.ToList())
            {
                Disconnect(nodeSequence, this);
            }
        }

        ~RGLNodeSequence()
        {
            Clear();
        }
    }
}