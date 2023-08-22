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

public static class SensorFactory
{
    public static Sensor<GnssInsSensorOptions> CreateGnssInsSensor()
    {
        var sensor = new Sensor<GnssInsSensorOptions>(SensorType.GNSS_INS)
        {
            IsActive = false,
            SensorLink = "",
            TopicNamespace = "",
            Translation = new Vector3(),
            Rotation = new Vector3()
        };
        return sensor;
    }

    public static Sensor<LidarSensorOptions> CreateLidarSensor()
    {
        var sensor = new Sensor<LidarSensorOptions>(SensorType.LIDAR)
        {
            IsActive = false,
            SensorLink = "",
            TopicNamespace = "",
            Translation = new Vector3(),
            Rotation = new Vector3()
        };
        return sensor;
    }

    public static Sensor<GroundTruthSensorOptions> CreateGroundTruthSensor()
    {
        var sensor = new Sensor<GroundTruthSensorOptions>(SensorType.GROUND_TRUTH)
        {
            IsActive = false,
            SensorLink = "",
            TopicNamespace = "",
            Translation = new Vector3(),
            Rotation = new Vector3()
        };
        return sensor;
    }

    public static Sensor<CameraSensorOptions> CreateCameraSensor()
    {
        var sensor = new Sensor<CameraSensorOptions>(SensorType.CAMERA)
        {
            IsActive = false,
            SensorLink = "",
            TopicNamespace = "",
            Translation = new Vector3(),
            Rotation = new Vector3()
        };
        return sensor;
    }

    public static Sensor<RadarSensorOptions> CreateRadarSensor()
    {
        var sensor = new Sensor<RadarSensorOptions>(SensorType.RADAR)
        {
            IsActive = false,
            SensorLink = "",
            TopicNamespace = "",
            Translation = new Vector3(),
            Rotation = new Vector3()
        };
        return sensor;
    }

    public static Sensor<RaptorSensorOptions> CreateRaptorSensor()
    {
        var sensor = new Sensor<RaptorSensorOptions>(SensorType.RAPTOR)
        {
            IsActive = true,
            SensorLink = "base_link",
            TopicNamespace = "",
            Translation = new Vector3(),
            Rotation = new Vector3(),
            Options = new RaptorSensorOptions
            {
                Model = SensorModel.Generic.GENERIC,
                IsHotStart = false,
                CanMode = true,
                CanInTopic = "to_can_bus",
                CanOutTopic = "from_can_bus"
            }
        };
        return sensor;
    }
}