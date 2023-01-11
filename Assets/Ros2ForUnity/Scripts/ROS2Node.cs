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

using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace ROS2
{

/// <summary>
/// A class representing a ros2 node. Multiple nodes can be used. Node can be removed by GC when not used anymore,
/// but will also be removed properly with Ros2cs Shutdown, which ROS2 for Unity performs on application quit
/// The node should be constructed through ROS2UnityComponent class, which also handles spinning
/// </summary>
public class ROS2Node
{
    internal INode node;
    public ROS2Clock clock;
    public string name;

    // Use ROS2UnityComponent to create a node
    internal ROS2Node(string unityROS2NodeName = "unity_ros2_node")
    {
        name = unityROS2NodeName;
        node = Ros2cs.CreateNode(name);
        clock = new ROS2Clock();
    }

    ~ROS2Node()
    {
        Ros2cs.RemoveNode(node);
    }

    private static void ThrowIfUninitialized(string callContext)
    {
        if (!Ros2cs.Ok())
        {
            throw new InvalidOperationException("Ros2 For Unity is not initialized, can't " + callContext);
        }
    }

    /// <summary>
    /// Create a publisher with QoS suitable for sensor data
    /// </summary>
    /// <returns>The publisher</returns>
    /// <param name="topicName">topic that will be used for publishing</param>
    public Publisher<T> CreateSensorPublisher<T>(string topicName) where T : Message, new()
    {
        QualityOfServiceProfile sensorProfile = new QualityOfServiceProfile(QosPresetProfile.SENSOR_DATA);
        return CreatePublisher<T>(topicName, sensorProfile);
    }

    /// <summary>
    /// Create a publisher with indicated QoS.
    /// </summary>
    /// <returns>The publisher</returns>
    /// <param name="topicName">topic that will be used for publishing</param>
    /// <param name="qos">QoS for publishing. If no QoS is selected, it will default to reliable, keep 10 last</param>
    public Publisher<T> CreatePublisher<T>(string topicName, QualityOfServiceProfile qos = null) where T : Message, new()
    {
        ThrowIfUninitialized("create publisher");
        return node.CreatePublisher<T>(topicName, qos);
    }

    /// <summary>
    /// Create a subscription
    /// </summary>
    /// <returns>The subscription</returns>
    /// <param name="topicName">topic to subscribe to</param>
    /// <param name="qos">QoS for subscription. If no QoS is selected, it will default to reliable, keep 10 last</param>
    public Subscription<T> CreateSubscription<T>(string topicName, Action<T> callback,
        QualityOfServiceProfile qos = null) where T : Message, new()
    {
        if (qos == null)
        {
            qos = new QualityOfServiceProfile(QosPresetProfile.DEFAULT);
        }
        ThrowIfUninitialized("create subscription");
        return node.CreateSubscription<T>(topicName, callback, qos);
    }


    /// <summary>
    /// Remove existing subscription (returned earlier with CreateSubscription)
    /// </summary>
    /// <returns>The whether subscription was found (e. g. false if removed earlier elsewhere) </returns>
    /// <param name="subscription">subscrition to remove, returned from CreateSubscription</param>
    public bool RemoveSubscription<T>(ISubscriptionBase subscription)
    {
        ThrowIfUninitialized("remove subscription");
        return node.RemoveSubscription(subscription);
    }

    /// <summary>
    /// Remove existing publisher
    /// </summary>
    /// <returns>The whether publisher was found (e. g. false if removed earlier elsewhere) </returns>
    /// <param name="publisher">publisher to remove, returned from CreatePublisher or CreateSensorPublisher</param>
    public bool RemovePublisher<T>(IPublisherBase publisher)
    {
        ThrowIfUninitialized("remove publisher");
        return node.RemovePublisher(publisher);
    }
}

}  // namespace ROS2
