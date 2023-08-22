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
using novatel_oem7_msgs.msg;

namespace Autonoma
{
public class RawimuPublisher : Publisher<RAWIMU>
{
    public string modifiedRosNamespace = "/novatel_bottom";
    public string modifiedTopicName = "/rawimu";
    public float modifiedFrequency = 125f;
    public string modifiedFrameId = "";
    public void getPublisherParams()
    {
        // get things from sensor assigned by ui to the sensor
    }
    protected override void Start()
    {
        getPublisherParams();
        this.rosNamespace = modifiedRosNamespace;
        this.topicName = modifiedTopicName;
        this.frequency = modifiedFrequency; // Hz
        this.frameId = modifiedFrameId;
        base.Start();
    }
    public ImuSimulator imuSim;
    public override void fillMsg()
    {
        msg.Linear_acceleration = new geometry_msgs.msg.Vector3();
        msg.Linear_acceleration.X = imuSim.imuAccel.x;
        msg.Linear_acceleration.Y = imuSim.imuAccel.y;
        msg.Linear_acceleration.Z = imuSim.imuAccel.z;

        msg.Angular_velocity = new geometry_msgs.msg.Vector3();
        msg.Angular_velocity.X = imuSim.imuGyro.x;
        msg.Angular_velocity.Y = imuSim.imuGyro.y;
        msg.Angular_velocity.Z = imuSim.imuGyro.z;
    }
} // end of class
} // end of autonoma namespace