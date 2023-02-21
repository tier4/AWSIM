// Copyright 2019-2021 Robotec.ai.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using UnityEngine;
using System;
using System.Collections.Generic;
using System.Threading;
using ROS2;

namespace ROS2
{

/// <summary>
/// The principal MonoBehaviour class for handling ros2 nodes and executables.
/// Use this to create ros2 node, check ros2 status.
/// Spins and executes actions (e. g. clock, sensor publish triggers) in a dedicated thread
/// TODO: this is meant to be used as a one-of (a singleton). Enforce. However, things should work
/// anyway with more than one since the underlying library can handle multiple init and shutdown calls,
/// and does node name uniqueness check independently.
/// </summary>
public class ROS2UnityComponent : MonoBehaviour
{
    private ROS2ForUnity ros2forUnity;
    private List<ROS2Node> nodes;
    private List<INode> ros2csNodes; // For performance in spinning
    private List<Action> executableActions;
    private bool initialized = false;
    private bool quitting = false;
    private int interval = 2;  // Spinning / executor interval in ms
    private object mutex = new object();
    private double spinTimeout = 0.0001;

    public bool Ok()
    {
        lock (mutex)
        {
            if (ros2forUnity == null)
                LazyConstruct();
            return (nodes != null && ros2forUnity.Ok());
        }
    }

    private void LazyConstruct()
    {
        lock (mutex)
        {        
            if (ros2forUnity != null)
                return;

            ros2forUnity = new ROS2ForUnity();
            nodes = new List<ROS2Node>();
            ros2csNodes = new List<INode>();
            executableActions = new List<Action>();
        }
    }

    void Start()
    {
        LazyConstruct();
    }

    public ROS2Node CreateNode(string name)
    {
        LazyConstruct();

        lock (mutex)
        {
            foreach (ROS2Node n in nodes)
            {  // Assumed to be a rare operation on rather small (<1k) list
                if (n.name == name)
                {
                    throw new InvalidOperationException("Cannot create node " + name + ". A node with this name already exists!");
                }
            }
            ROS2Node node = new ROS2Node(name);
            nodes.Add(node);
            ros2csNodes.Add(node.node);
            return node;
        }
    }

    public void RemoveNode(ROS2Node node)
    {
        lock (mutex)
        {
            ros2csNodes.Remove(node.node);
            nodes.Remove(node); //Node will be later deleted if unused, by GC
        }
    }

    /// <summary>
    /// Works as a simple executor registration analogue. These functions will be called with each Tick()
    /// Actions need to take care of correct call resolution by checking in their body (TODO)
    /// Make sure actions are lightweight (TODO - separate out threads for spinning and executables?)
    /// </summary>
    public void RegisterExecutable(Action executable)
    {
        LazyConstruct();

        lock (mutex)
        {
            executableActions.Add(executable);
        }
    }

    public void UnregisterExecutable(Action executable)
    {
        lock (mutex)
        {
            executableActions.Remove(executable);
        }
    }

    /// <summary>
    /// "Executor" thread will tick all clocks and spin the node
    /// </summary>
    private void Tick()
    {
        while (!quitting)
        {
            if (Ok())
            {
                lock (mutex)
                {
                    foreach (Action action in executableActions)
                    {
                        action();
                    }
                    Ros2cs.SpinOnce(ros2csNodes, spinTimeout);
                }
            }
            Thread.Sleep(interval);
        }
    }

    void FixedUpdate()
    {
        if (!initialized)
        {
            Thread publishThread = new Thread(() => Tick());
            publishThread.Start();
            initialized = true;
        }
    }

    void OnApplicationQuit()
    {
        quitting = true;
        ros2forUnity.DestroyROS2ForUnity();
    }
}

}  // namespace ROS2
