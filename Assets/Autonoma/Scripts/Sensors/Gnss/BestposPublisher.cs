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
public class BestposPublisher : Publisher<BESTPOS>
{
    public string modifiedRosNamespace = "/novatel_bottom";
    public string modifiedTopicName = "/bestpos";
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
        msg.Pos_type = new PositionOrVelocityType();
        msg.Pos_type.Type = 50;
        msg.Lat = gnssSim.llh[0];
        msg.Lon = gnssSim.llh[1];
        msg.Hgt = gnssSim.llh[2];
        msg.Undulation = 0.0f;
        msg.Datum_id = 0;
        msg.Lat_stdev = 0.01f;
        msg.Lon_stdev = 0.01f;
        msg.Hgt_stdev = 0.01f;
        msg.Diff_age = 0.0f;
        msg.Sol_age = 0.0f;
        msg.Num_svs = 15;
        msg.Num_sol_svs = 12;
        msg.Num_sol_l1_svs = 6;
        msg.Num_sol_multi_svs = 6;
        msg.Reserved = 0;
        msg.Ext_sol_stat = new BestExtendedSolutionStatus();
        msg.Galileo_beidou_sig_mask = 0;
        msg.Gps_glonass_sig_mask = 0;
    }
} // end of class
} // end of autonoma namespace