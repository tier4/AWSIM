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
public class BestvelPublisher : Publisher<BESTVEL>
{
    public string modifiedRosNamespace = "/novatel_bottom";
    public string modifiedTopicName = "/bestvel";
    public float modifiedFrequency = 20f;
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
    public GnssSimulator gnssSim;
    public override void fillMsg()
    {
        msg.Vel_type = new PositionOrVelocityType();
        msg.Vel_type.Type = 50;
        msg.Latency = 0.0f;
        msg.Diff_age = 0.0f;
        msg.Hor_speed = Mathf.Sqrt(Mathf.Pow(gnssSim.vE,2) + Mathf.Pow(gnssSim.vN,2) );
        msg.Trk_gnd = (Mathf.Atan2(gnssSim.vE,gnssSim.vN)*180f/Mathf.PI) % 360;
        msg.Ver_speed = gnssSim.vU;
        msg.Reserved = 0.0f;
    }
} // end of class
} // end of autonoma namespace