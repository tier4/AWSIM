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
using autonoma_msgs.msg;

namespace Autonoma
{
public class RaceControlDataPublisher : Publisher<autonoma_msgs.msg.RaceControl>
{
    public string modifiedRosNamespace = "";
    public string modifiedTopicName = "/race_control";
    public float modifiedFrequency = 10f;
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
    public RaceControlData raceControlData;
    public override void fillMsg()
    {
        msg.Track_flag = raceControlData.rc.TrackFlag;
        msg.Veh_flag = raceControlData.rc.VehicleFlag;
    }
} // end of class
} // end of autonoma namespace