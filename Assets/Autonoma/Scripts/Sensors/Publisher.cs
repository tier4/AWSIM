/* 
Copyright 2023 Autonoma, Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at:

    http://www.apache.org/licenses/LICENSE-2.0

The software is provided "AS IS", WITHOUT WARRANTY OF ANY KIND, 
express or implied. In no event shall the authors or copyright 
holders be liable for any claim, damages or other liability, 
whether in action of contract, tort or otherwise, arising from, 
out of or in connection with the software or the use of the software.
*/
using UnityEngine;
using ROS2;
using AWSIM;

namespace Autonoma
{
public class Publisher<T> : MonoBehaviour, IPublisherBase where T : ROS2.Message, new()
{
    public virtual string rosNamespace { get; set;}
    public virtual string topicName  { get; set;}
    public virtual float frequency { get; set;} //Hz
    public virtual string frameId  { get; set;}
    IPublisher<T> publisher;
    public T msg;
    public QoSSettings qosSettings = new QoSSettings()
    {
        ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
        DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
        HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
        Depth = 10,
    };
    private float timer;
    private string fullTopicName;
    public virtual void fillMsg(){}
    protected virtual void Start()
    {
        msg = new T();
        fullTopicName = rosNamespace + topicName;
        // Create publisher.
        var qos = qosSettings.GetQoSProfile();
        publisher = SimulatorROS2Node.CreatePublisher<T>(fullTopicName, qos);
    }
    public void FixedUpdate()
    {
        timer += Time.fixedDeltaTime;
        if ( timer > (1.0f/frequency) )
        {
            fillMsg();
            // Update msg header.
            var header = msg as MessageWithHeader;
            SimulatorROS2Node.UpdateROSTimestamp(ref header);
            // Publish to ROS2.
            publisher.Publish(msg);
            timer = 0.000f;
        }
    }
    void OnDestroy()
    {
        SimulatorROS2Node.RemovePublisher<T>(publisher);
    }

    public void ToggleActive(bool isActive)
    {
        this.gameObject.SetActive(isActive);
    }
}
} // end of autonoma namespace