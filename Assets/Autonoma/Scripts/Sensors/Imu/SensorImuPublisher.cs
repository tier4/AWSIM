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
using sensor_msgs.msg;

namespace Autonoma
{
public class SensorImuPublisher : Publisher<Imu>
{
    public string modifiedRosNamespace = "";
    public string modifiedTopicName = "/imu";
    public float modifiedFrequency = 100f;
    public string modifiedFrameId = "";
    public float linear_acceleration_covariance = 0.0009f;
    public float angular_velocity_covariance = 0.00035f;
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
        msg.Linear_acceleration_covariance[0] = linear_acceleration_covariance;
        msg.Linear_acceleration_covariance[4] = linear_acceleration_covariance;
        msg.Linear_acceleration_covariance[8] = linear_acceleration_covariance;

        msg.Angular_velocity = new geometry_msgs.msg.Vector3();
        msg.Angular_velocity.X = imuSim.imuGyro.x;
        msg.Angular_velocity.Y = imuSim.imuGyro.y;
        msg.Angular_velocity.Z = imuSim.imuGyro.z;
        msg.Angular_velocity_covariance[0] = angular_velocity_covariance;
        msg.Angular_velocity_covariance[4] = angular_velocity_covariance;
        msg.Angular_velocity_covariance[8] = angular_velocity_covariance;
    }
}
}