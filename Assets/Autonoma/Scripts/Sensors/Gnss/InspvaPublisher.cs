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
public class InspvaPublisher : Publisher<INSPVA>
{
    public string modifiedRosNamespace = "/novatel_bottom";
    public string modifiedTopicName = "/inspva";
    public float modifiedFrequency = 100f;
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
    public ImuSimulator imuSim;
    public override void fillMsg()
    {
        msg.Latitude = gnssSim.lat;
        msg.Longitude = gnssSim.lon;
        msg.Height = gnssSim.height;
        msg.East_velocity = gnssSim.vE;
        msg.North_velocity = gnssSim.vN;
        msg.Up_velocity = gnssSim.vU;
        msg.Roll = imuSim.imuAngle[0]; //deg
        msg.Pitch = imuSim.imuAngle[1];
        msg.Azimuth = imuSim.imuAngle[2];
    }
} // end of class
} // end of autonoma namespace