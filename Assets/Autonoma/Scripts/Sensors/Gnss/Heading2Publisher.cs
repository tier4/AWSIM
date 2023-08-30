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
using VehicleDynamics;
using novatel_oem7_msgs.msg;

namespace Autonoma
{
public class Heading2Publisher : Publisher<HEADING2>
{
    public string modifiedRosNamespace = "/novatel_bottom";
    public string modifiedTopicName = "/heading2";
    public float modifiedFrequency = 1f;
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
    public Heading2Simulator heading2Sim;
    public override void fillMsg()
    {
        msg.Pos_type = new PositionOrVelocityType();
        msg.Pos_type.Type = 50;
        msg.Heading = heading2Sim.heading2;
        msg.Heading_stdev = 2.13f;
        msg.Num_sv_tracked = 15;
        msg.Num_sv_in_sol = 12;
        msg.Num_sv_obs = 12;
        msg.Num_sv_multi = 6;
        msg.Reserved = 0;
        msg.Ext_sol_status = new BestExtendedSolutionStatus();
        msg.Galileo_beidou_sig_mask = 0;
        msg.Gps_glonass_sig_mask = 0;
    }
} // end of class
} // end of autonoma namespace