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

        //// NODESEQUENCES OPERATIONS ////
        public static void Connect(RGLNodeSequence parent, RGLNodeSequence child)
        {
            if (parent.childs.Contains(child) || child.parents.Contains(parent))
            {
                throw new RGLException("Attempted to connect NodeSequences twice!");
            }

            // Connect nodes in RGL if NodeSequences have active nodes.
            // They will be connected in RGL when activating the node or adding the new one.
            if (parent.GetLastNodeOrNull(true) != null && child.GetFirstNodeOrNull(true) != null)
            {
                RGLNativeAPI.GraphNodeAddChild(parent.GetLastNodeOrNull(true).Node, child.GetFirstNodeOrNull(true).Node);
            }

            parent.childs.Add(child);
            child.parents.Add(parent);
        }

        public static void Disconnect(RGLNodeSequence parent, RGLNodeSequence child)
        {
            if (!parent.childs.Contains(child) || !child.parents.Contains(parent))
            {
                throw new RGLException("Attempted to disconnect NodeSequences that are not connected!");
            }

            // Disconnect RGL nodes if NodeSequences have active nodes.
            // Otherwise they are already disconnected in RGL.
            if (parent.GetLastNodeOrNull(true) != null && child.GetFirstNodeOrNull(true) != null)
            {
                RGLNativeAPI.GraphNodeRemoveChild(parent.GetLastNodeOrNull(true).Node, child.GetFirstNodeOrNull(true).Node);
            }

            parent.childs.Remove(child);
            child.parents.Remove(parent);
        }

        public static bool AreConnected(RGLNodeSequence parent, RGLNodeSequence child)
        {
            return parent.childs.Contains(child) && child.parents.Contains(parent);
        }

        //// ADD NODES ////
        public RGLNodeSequence AddNodeRaysFromMat3x4f(string identifier, Matrix4x4[] rays)
        {
            CheckNodeNotExist(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodeRaysFromMat3x4f(ref handle.Node, rays);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.RAYS_FROM_MAT3X4F;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodeRaysSetRange(string identifier, Vector2[] ranges)
        {
            CheckNodeNotExist(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodeRaysSetRange(ref handle.Node, ranges);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.RAYS_SET_RANGE;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodeRaysSetRingIds(string identifier, int[] ringIds)
        {
            CheckNodeNotExist(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodeRaysSetRingIds(ref handle.Node, ringIds);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.RAYS_SET_RING_IDS;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodeRaysSetTimeOffsets(string identifier, float[] offsets)
        {
            CheckNodeNotExist(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodeRaysSetTimeOffsets(ref handle.Node, offsets);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.RAYS_SET_TIME_OFFSETS;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodeRaysTransform(string identifier, Matrix4x4 transform)
        {
            CheckNodeNotExist(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodeRaysTransform(ref handle.Node, transform);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.RAYS_TRANSFORM;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodeRaytrace(string identifier)
        {
            CheckNodeNotExist(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodeRaytrace(ref handle.Node);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.RAYTRACE;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodePointsTransform(string identifier, Matrix4x4 transform)
        {
            CheckNodeNotExist(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodePointsTransform(ref handle.Node, transform);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.POINTS_TRANSFORM;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodePointsCompact(string identifier)
        {
            CheckNodeNotExist(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodePointsCompact(ref handle.Node);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.POINTS_COMPACT;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodePointsDownsample(string identifier, Vector3 leafDims)
        {
            CheckNodeNotExist(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodePointsDownSample(ref handle.Node, leafDims);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.POINTS_DOWNSAMPLE;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodePointsTemporalMerge(string identifier, RGLField[] fields)
        {
            CheckNodeNotExist(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodePointsTemporalMerge(ref handle.Node, fields);
            handle.Identifier = identifier;
            handle.Type = RGLNodeType.POINTS_TEMPORAL_MERGE;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodePointsYield(string identifier, RGLField field)
        {
            CheckNodeNotExist(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodePointsYield(ref handle.Node, new [] {field});
            handle.Type = RGLNodeType.POINTS_YIELD;
            handle.OutputField = field;
            handle.Identifier = identifier;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodePointsFormat(string identifier, RGLField[] fields)
        {
            CheckNodeNotExist(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodePointsFormat(ref handle.Node, fields);
            handle.Type = RGLNodeType.POINTS_FORMAT;
            handle.OutputField = RGLField.DYNAMIC_FORMAT;
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
            CheckNodeNotExist(identifier);
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

        public RGLNodeSequence AddNodePointsUdpPublish(string identifier, RGLLidarModel lidarModel, RGLUdpOptions udpOptions, string deviceIp, string destIp, int destPort)
        {
            CheckNodeNotExist(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodePointsUdpPublish(ref handle.Node, lidarModel, udpOptions, deviceIp, destIp, destPort);
            handle.Type = RGLNodeType.POINTS_UDP_PUBLISH;
            handle.Identifier = identifier;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodeGaussianNoiseAngularRay(string identifier, float mean, float stDev)
        {
            CheckNodeNotExist(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodeGaussianNoiseAngularRay(ref handle.Node, mean, stDev);
            handle.Type = RGLNodeType.GAUSSIAN_NOISE_ANGULAR_RAY;
            handle.Identifier = identifier;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodeGaussianNoiseAngularHitpoint(string identifier, float mean, float stDev)
        {
            CheckNodeNotExist(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodeGaussianNoiseAngularHitpoint(ref handle.Node, mean, stDev);
            handle.Type = RGLNodeType.GAUSSIAN_NOISE_ANGULAR_HITPOINT;
            handle.Identifier = identifier;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodeGaussianNoiseDistance(string identifier, float mean, float stDev, float stDevRisePerMeter)
        {
            CheckNodeNotExist(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodeGaussianNoiseDistance(ref handle.Node, mean, stDev, stDevRisePerMeter);
            handle.Type = RGLNodeType.GAUSSIAN_NOISE_DISTANCE;
            handle.Identifier = identifier;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodePointsRemoveGround(string identifier, float groundAngleThreshold, float groundDistanceThreshold, float groundFilterDistance)
        {
            CheckNodeNotExist(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodePointsRemoveGround(ref handle.Node, groundAngleThreshold, groundDistanceThreshold, groundFilterDistance);
            handle.Type = RGLNodeType.POINTS_REMOVE_GROUND;
            handle.Identifier = identifier;
            AddNode(handle);
            return this;
        }

        public RGLNodeSequence AddNodePointsRadarPostprocess(string identifier, float distanceSeparation, float azimuthSeparation)
        {
            CheckNodeNotExist(identifier);
            RGLNodeHandle handle = new RGLNodeHandle();
            RGLNativeAPI.NodePointsRadarPostprocess(ref handle.Node, distanceSeparation, azimuthSeparation);
            handle.Type = RGLNodeType.POINTS_RADAR_POSTPROCESS;
            handle.Identifier = identifier;
            AddNode(handle);
            return this;
        }

        //// UPDATE NODES ////
        public RGLNodeSequence UpdateNodeRaysFromMat3x4f(string identifier, Matrix4x4[] rays)
        {
            RGLNodeHandle handle = ValidateNode(identifier, RGLNodeType.RAYS_FROM_MAT3X4F);
            RGLNativeAPI.NodeRaysFromMat3x4f(ref handle.Node, rays);
            return this;
        }

        public RGLNodeSequence UpdateNodeRaysSetRange(string identifier, Vector2[] ranges)
        {
            RGLNodeHandle handle = ValidateNode(identifier, RGLNodeType.RAYS_SET_RANGE);
            RGLNativeAPI.NodeRaysSetRange(ref handle.Node, ranges);
            return this;
        }

        public RGLNodeSequence UpdateNodeRaysSetRingIds(string identifier, int[] ringIds)
        {
            RGLNodeHandle handle = ValidateNode(identifier, RGLNodeType.RAYS_SET_RING_IDS);
            RGLNativeAPI.NodeRaysSetRingIds(ref handle.Node, ringIds);
            return this;
        }

        public RGLNodeSequence UpdateNodeRaysTimeOffsets(string identifier, float[] offsets)
        {
            RGLNodeHandle handle = ValidateNode(identifier, RGLNodeType.RAYS_SET_TIME_OFFSETS);
            RGLNativeAPI.NodeRaysSetTimeOffsets(ref handle.Node, offsets);
            return this;
        }

        public RGLNodeSequence UpdateNodeRaysTransform(string identifier, Matrix4x4 transform)
        {
            RGLNodeHandle handle = ValidateNode(identifier, RGLNodeType.RAYS_TRANSFORM);
            RGLNativeAPI.NodeRaysTransform(ref handle.Node, transform);
            return this;
        }

        public RGLNodeSequence UpdateNodeRaytrace(string identifier)
        {
            RGLNodeHandle handle = ValidateNode(identifier, RGLNodeType.RAYTRACE);
            RGLNativeAPI.NodeRaytrace(ref handle.Node);
            return this;
        }

        public RGLNodeSequence UpdateNodeRaytrace(string identifier, Vector3 linearVelocity, Vector3 angularVelocity, bool applyRayDistortion)
        {
            RGLNodeHandle handle = ValidateNode(identifier, RGLNodeType.RAYTRACE);
            RGLNativeAPI.NodeRaytrace(ref handle.Node, linearVelocity, angularVelocity, applyRayDistortion);
            return this;
        }

        public RGLNodeSequence UpdateNodePointsTransform(string identifier, Matrix4x4 transform)
        {
            RGLNodeHandle handle = ValidateNode(identifier, RGLNodeType.POINTS_TRANSFORM);
            RGLNativeAPI.NodePointsTransform(ref handle.Node, transform);
            return this;
        }

        public RGLNodeSequence UpdateNodePointsDownsample(string identifier, Vector3 leafDims)
        {
            RGLNodeHandle handle = ValidateNode(identifier, RGLNodeType.POINTS_DOWNSAMPLE);
            RGLNativeAPI.NodePointsDownSample(ref handle.Node, leafDims);
            return this;
        }

        public RGLNodeSequence UpdateNodePointsTemporalMerge(string identifier, RGLField[] fields)
        {
            RGLNodeHandle handle = ValidateNode(identifier, RGLNodeType.POINTS_TEMPORAL_MERGE);
            RGLNativeAPI.NodePointsTemporalMerge(ref handle.Node, fields);
            return this;
        }

        public RGLNodeSequence UpdateNodePointsUdpPublish(string identifier, RGLLidarModel lidarModel, RGLUdpOptions udpOptions, string deviceIp, string destIp, int destPort)
        {
            RGLNodeHandle handle = ValidateNode(identifier, RGLNodeType.POINTS_UDP_PUBLISH);
            RGLNativeAPI.NodePointsUdpPublish(ref handle.Node, lidarModel, udpOptions, deviceIp, destIp, destPort);
            return this;
        }

        public RGLNodeSequence UpdateNodeGaussianNoiseAngularRay(string identifier, float mean, float stDev)
        {
            RGLNodeHandle handle = ValidateNode(identifier, RGLNodeType.GAUSSIAN_NOISE_ANGULAR_RAY);
            RGLNativeAPI.NodeGaussianNoiseAngularRay(ref handle.Node, mean, stDev);
            return this;
        }

        public RGLNodeSequence UpdateNodeGaussianNoiseAngularHitpoint(string identifier, float mean, float stDev)
        {
            RGLNodeHandle handle = ValidateNode(identifier, RGLNodeType.GAUSSIAN_NOISE_ANGULAR_HITPOINT);
            RGLNativeAPI.NodeGaussianNoiseAngularHitpoint(ref handle.Node, mean, stDev);
            return this;
        }

        public RGLNodeSequence UpdateNodeGaussianNoiseDistance(string identifier, float mean, float stDev, float stDevRisePerMeter)
        {
            RGLNodeHandle handle = ValidateNode(identifier, RGLNodeType.GAUSSIAN_NOISE_DISTANCE);
            RGLNativeAPI.NodeGaussianNoiseDistance(ref handle.Node, mean, stDev, stDevRisePerMeter);
            return this;
        }

        public RGLNodeSequence UpdateNodePointsRemoveGround(string identifier, float groundAngleThreshold, float groundDistanceThreshold, float groundFilterDistance)
        {
            RGLNodeHandle handle = ValidateNode(identifier, RGLNodeType.POINTS_REMOVE_GROUND);
            RGLNativeAPI.NodePointsRemoveGround(ref handle.Node, groundAngleThreshold, groundDistanceThreshold, groundFilterDistance);
            return this;
        }

        public RGLNodeSequence UpdateNodePointsRadarPostprocess(string identifier, float distanceSeparation, float azimuthSeparation)
        {
            RGLNodeHandle handle = ValidateNode(identifier, RGLNodeType.POINTS_RADAR_POSTPROCESS);
            RGLNativeAPI.NodePointsRadarPostprocess(ref handle.Node, distanceSeparation, azimuthSeparation);
            return this;
        }

        //// NODESEQUENCE OPERATIONS ////
        public int GetResultData<T>(ref T[] data) where T : unmanaged
        {
            RGLNodeHandle handle = ValidateOutputNode();
            unsafe
            {
                return RGLNativeAPI.GraphGetResult<T>(handle.Node, handle.OutputField, ref data, sizeof(T));
            }
        }

        public int GetResultDataRaw(ref byte[] data, int expectedPointSize)
        {
            RGLNodeHandle handle = ValidateOutputNode();
            return RGLNativeAPI.GraphGetResult<byte>(handle.Node, handle.OutputField, ref data, expectedPointSize);
        }

        public int GetPointCloudCount(string identifier = null, RGLField field = RGLField.XYZ_VEC3_F32)
        {
            RGLNodeHandle handle = identifier == null ? GetLastNodeOrNull(true) : ValidateNode(identifier);
            if (handle == null)
            {
                throw new RGLException("Attempted to get point cloud count from empty NodeSequence!");
            }
            return RGLNativeAPI.GraphGetResultSize(handle.Node, field);
        }

        public void SavePcdFile(string outFilepath)
        {
            RGLNodeHandle lastNode = GetLastNodeOrNull(true);
            if (lastNode == null)
            {
                throw new RGLException("Attempted to save PCD file from empty NodeSequence!");
            }
            RGLNativeAPI.GraphSavePcdFile(lastNode.Node, outFilepath);
        }

        public void Run()
        {
            RGLNodeHandle handle = GetFirstNodeOrNull(true);
            if (handle == null)
            {
                throw new RGLException("Attempted to run empty NodeSequence!");
            }
            RGLNativeAPI.GraphRun(handle.Node);
        }

        public void RemoveNode(string identifier)
        {
            RGLNodeHandle node = ValidateNode(identifier);
            if (node.Connected)
            {
                DisconnectNode(node);
            }
            RGLNativeAPI.GraphDestroy(node.Node);
            nodes.Remove(node);
        }

        public void Clear()
        {
            DisconnectAllChilds();
            DisconnectAllParents();

            foreach (var node in nodes.ToArray())
            {
                RemoveNode(node.Identifier);
            }
        }

        public bool IsActive(string identifier)
        {
            return ValidateNode(identifier).Connected;
        }

        public void SetActive(string identifier, bool active)
        {
            RGLNodeHandle node = ValidateNode(identifier);
            if (active && !node.Connected)
            {
                ConnectNode(node);
                return;
            }
            if (!active && node.Connected)
            {
                DisconnectNode(node);
            }
        }

        public void SetPriority(string identifier, int priority)
        {
            RGLNodeHandle node = ValidateNode(identifier);
            RGLNativeAPI.GraphNodeSetPriority(node.Node, priority);
        }

        //// PRIVATE HELPERS ////
        // Throws an exception when node with provided identifier already exists in this NodeSequence
        private void CheckNodeNotExist(string identifier)
        {
            var nodeFilter = nodes.Where(n => n.Identifier == identifier);
            if (nodeFilter.Count() != 0)
            {
                throw new RGLException($"Node with identifier '{identifier}' already exists!");
            }
        }

        // Returns RGLNodeHandle to the node with provided identifier.
        // Throws an exception when node was not found or type of the node mismatch.
        private RGLNodeHandle ValidateNode(string identifier, RGLNodeType desiredType = RGLNodeType.ANY)
        {
            var nodeFilter = nodes.Where(n => n.Identifier == identifier);
            if (nodeFilter.Count() != 1)
            {
                throw new RGLException($"Attempted to access node '{identifier}' but it was not found!");
            }
            if (desiredType != RGLNodeType.ANY && nodeFilter.First().Type != desiredType)
            {
                throw new RGLException($"Attempted to access node '{identifier}' but node type mismatch!");
            }
            return nodeFilter.First();
        }

        // Returns RGLNodeHandle to the output node in this NodeSequence.
        // Throws an exception when output node was not found. 
        private RGLNodeHandle ValidateOutputNode()
        {
            RGLNodeHandle outputNode = GetLastNodeOrNull(true);
            if (outputNode == null)
            {
                throw new RGLException("Attempted to get result data from empty NodeSequence!");
            }
            if (outputNode.OutputField == RGLField.UNKNOWN)
            {
                throw new RGLException("Attempted to get result data but output node was not found on the last position in the NodeSequence!");
            }
            return outputNode;
        }

        private void AddNode(RGLNodeHandle nodeHandle)
        {
            nodes.Add(nodeHandle);
            ConnectNode(nodeHandle);
        }

        private void ConnectNode(RGLNodeHandle nodeToConnect)
        {
            if (nodeToConnect.Connected)
            {
                throw new RGLException($"Attempted to connect node '{nodeToConnect.Identifier}' twice!");
            }

            // Collect previous and next nodes that are connected
            RGLNodeHandle[] prevNodes = GetPreviousNodes(nodeToConnect, true);
            RGLNodeHandle[] nextNodes = GetNextNodes(nodeToConnect, true);

            // Nodes are connected with each other. Need to disconnect them.
            if (prevNodes.Length != 0 && nextNodes.Length != 0)
            {
                foreach (RGLNodeHandle prevNode in prevNodes)
                {
                    foreach (RGLNodeHandle nextNode in nextNodes)
                    {
                        RGLNativeAPI.GraphNodeRemoveChild(prevNode.Node, nextNode.Node);
                    }
                }
            }

            // Connect nodeToConnect between prevNodes and nextNodes
            foreach (RGLNodeHandle prevNode in prevNodes)
            {
                RGLNativeAPI.GraphNodeAddChild(prevNode.Node, nodeToConnect.Node);
            }
            foreach (RGLNodeHandle nextNode in nextNodes)
            {
                RGLNativeAPI.GraphNodeAddChild(nodeToConnect.Node, nextNode.Node);
            }

            nodeToConnect.Connected = true;
        }

        private void DisconnectNode(RGLNodeHandle nodeToDisconnect)
        {
            if (!nodeToDisconnect.Connected)
            {
                throw new RGLException($"Attempted to disconnect node '{nodeToDisconnect.Identifier}' that is not connected!");
            }

            // Collect previous and next nodes that are connected
            RGLNodeHandle[] prevNodes = GetPreviousNodes(nodeToDisconnect, true);
            RGLNodeHandle[] nextNodes = GetNextNodes(nodeToDisconnect, true);

            // Disconnect nodeToDisconnect with prevNodes and nextNodes
            foreach (RGLNodeHandle prevNode in prevNodes)
            {
                RGLNativeAPI.GraphNodeRemoveChild(prevNode.Node, nodeToDisconnect.Node);
            }
            foreach (RGLNodeHandle nextNode in nextNodes)
            {
                RGLNativeAPI.GraphNodeRemoveChild(nodeToDisconnect.Node, nextNode.Node);
            }

            bool anyNodeRemained = false;
            foreach (RGLNodeHandle node in prevNodes.ToList().Concat(nextNodes.ToList()))
            {
                if (nodes.Contains(node))
                {
                    anyNodeRemained = true;
                    break;
                }
            }

            // Connect nodes that have become adjacent
            // If there is no remaining nodes in this sequence - skip connecting
            if (prevNodes.Length != 0 && nextNodes.Length != 0 && anyNodeRemained)
            {
                foreach (RGLNodeHandle prevNode in prevNodes)
                {
                    foreach (RGLNodeHandle nextNode in nextNodes)
                    {
                        RGLNativeAPI.GraphNodeAddChild(prevNode.Node, nextNode.Node);
                    }
                }
            }

            nodeToDisconnect.Connected = false;
        }

        //// NODE GETTERS ////
        // Get first node in this NodeSequence or return null
        private RGLNodeHandle GetFirstNodeOrNull(bool mustBeConnected)
        {
            int idx = nodes.FindIndex(n => !mustBeConnected || n.Connected == true);
            return idx != -1 ? nodes[idx] : null;
        }

        // Get last node in this NodeSequence or return null
        private RGLNodeHandle GetLastNodeOrNull(bool mustBeConnected)
        {
            int idx = nodes.FindLastIndex(n => !mustBeConnected || n.Connected == true);
            return idx != -1 ? nodes[idx] : null;
        }

        // Get previous nodes including parent NodeSequences
        private RGLNodeHandle[] GetPreviousNodes(RGLNodeHandle referenceNode, bool mustBeConnected)
        {
            List<RGLNodeHandle> outNodes = new List<RGLNodeHandle>();
            int refNodeIdx = nodes.FindIndex(n => n.Identifier == referenceNode.Identifier);

            var prevNodesInThisSeq = nodes.GetRange(0, refNodeIdx);
            int prevNodeIdx = prevNodesInThisSeq.FindLastIndex(n => !mustBeConnected || n.Connected == true);

            if (prevNodeIdx == -1)  // This sequence doesn't contain any nodes. Search in parent sequences
            {
                foreach (RGLNodeSequence nodeSequence in parents.ToList())
                {
                    outNodes.Add(nodeSequence.GetLastNodeOrNull(mustBeConnected));
                }
                outNodes.RemoveAll(n => n == null);
            }
            else
            {
                outNodes.Add(prevNodesInThisSeq[prevNodeIdx]);
            }
            return outNodes.ToArray();
        }

        // Get next nodes including child NodeSequences
        private RGLNodeHandle[] GetNextNodes(RGLNodeHandle referenceNode, bool mustBeConnected)
        {
            List<RGLNodeHandle> outNodes = new List<RGLNodeHandle>();
            int refNodeIdx = nodes.FindIndex(n => n.Identifier == referenceNode.Identifier);

            List<RGLNodeHandle> nextNodesInThisSeq = new List<RGLNodeHandle>();
            if (refNodeIdx + 1 < nodes.Count)
            {
                nextNodesInThisSeq = nodes.GetRange(refNodeIdx + 1, nodes.Count - refNodeIdx - 1);
            }
            int nextNodeIdx = nextNodesInThisSeq.FindIndex(n => !mustBeConnected || n.Connected == true);

            if (nextNodeIdx == -1)  // This sequence doesn't contain any nodes. Search in child sequences
            {
                foreach (RGLNodeSequence nodeSequence in childs.ToList())
                {
                    outNodes.Add(nodeSequence.GetFirstNodeOrNull(mustBeConnected));
                }
                outNodes.RemoveAll(n => n == null);
            }
            else
            {
                outNodes.Add(nextNodesInThisSeq[nextNodeIdx]);
            }
            return outNodes.ToArray();
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
