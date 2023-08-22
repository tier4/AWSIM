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
using vectornav_msgs.msg;

namespace Autonoma
{
public class CommonGroupPublisher : Publisher<CommonGroup>
{
    public string modifiedRosNamespace = "/vectornav";
    public string modifiedTopicName = "/raw/common";
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
        // SensorMotion is ENU, RPY
        // VN is NED, YPR
        msg.Yawpitchroll.X = -imuSim.imuAngle.z;
        msg.Yawpitchroll.Y = -imuSim.imuAngle.x;
        msg.Yawpitchroll.Z = imuSim.imuAngle.y;
        
        Quaternion quat = Quaternion.Euler((float)msg.Yawpitchroll.X, 
            (float)msg.Yawpitchroll.Y, (float)msg.Yawpitchroll.Z);
        msg.Quaternion.X = quat.x;
        msg.Quaternion.Y = quat.y;
        msg.Quaternion.Z = quat.z;
        msg.Quaternion.W = quat.w;

        msg.Position.X = gnssSim.lat;
        msg.Position.Y = gnssSim.lon;
        msg.Position.Z = gnssSim.height;

        msg.Velocity.X = gnssSim.vN;
        msg.Velocity.Y = gnssSim.vE;
        msg.Velocity.Z = -gnssSim.vU;
        
        msg.Angularrate.X = -imuSim.imuGyro.z;
        msg.Angularrate.Y = -imuSim.imuGyro.x;
        msg.Angularrate.Z = imuSim.imuGyro.y;
        
        msg.Accel.X = imuSim.imuAccel.y;
        msg.Accel.Y = imuSim.imuAccel.x;
        msg.Accel.Z = -imuSim.imuAccel.z;
        
        msg.Imu_rate.X = -imuSim.imuGyro.z;
        msg.Imu_rate.Y = -imuSim.imuGyro.x;
        msg.Imu_rate.Z = imuSim.imuGyro.y;
        
        msg.Imu_accel.X = imuSim.imuAccel.y;
        msg.Imu_accel.Y = imuSim.imuAccel.x;
        msg.Imu_accel.Z = -imuSim.imuAccel.z;

        msg.Magpres_mag.X = Mathf.Cos(-imuSim.imuAngle.z);
        msg.Magpres_mag.Y = Mathf.Sin(-imuSim.imuAngle.z);
        msg.Magpres_mag.Z = 0.0f;

        msg.Magpres_temp = 49.7f;

        msg.Deltatheta_dtime = 1.0f/this.frequency;

        msg.Deltatheta_dtheta.X = msg.Imu_rate.X;
        msg.Deltatheta_dtheta.Y = msg.Imu_rate.Y;
        msg.Deltatheta_dtheta.Z = msg.Imu_rate.Z;
        
        msg.Deltatheta_dvel.X = msg.Imu_accel.X;
        msg.Deltatheta_dvel.Y = msg.Imu_accel.Y;
        msg.Deltatheta_dvel.Z = msg.Imu_accel.Z;

        msg.Insstatus.Mode = 2;
        msg.Insstatus.Gps_fix = true;
        msg.Insstatus.Time_error = false;
        msg.Insstatus.Imu_error = false;
        msg.Insstatus.Mag_pres_error = false;
        msg.Insstatus.Gps_error = false;
        msg.Insstatus.Gps_heading_ins = true;
        msg.Insstatus.Gps_compass = false;
    }
} // end of class
} // end of autonoma namespace