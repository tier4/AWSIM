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
using System.Collections.Generic;
using UnityEngine;

public class ScenarioObj
{
    public string Name;
    public int Version;
    public int NumCars;
    public int SelectedTrack;
    public bool HotStart;
    public List<Car> Cars;
    public ScenarioObj()
    {   
        Name = "";
        NumCars = 1;
        Cars = new List<Car>();
        SelectedTrack = 0;
    }
}

public interface ISensorOptions {}

public class GenericSensorOptions : ISensorOptions
{
    public SensorModel.Generic Model;
}

public class GnssInsSensorOptions : ISensorOptions
{
    public SensorModel.GnssIns Model;
    public float LatStdDev;
    public float LonStdDev;
    public float HgtStdDev;
    public float HeadingStdDev;
    public float AccelCovariance;
    public float GyroCovariance;
}

public class RaptorSensorOptions : ISensorOptions
{
    public SensorModel.Generic Model;
    public bool IsHotStart;
    public bool CanMode;
    public string CanOutTopic;
    public string CanInTopic;
}

public class LidarSensorOptions : ISensorOptions
{
    public SensorModel.Lidar Model;
}

public class CameraSensorOptions : ISensorOptions
{
    public SensorModel.Generic Model;
}

public class RadarSensorOptions : ISensorOptions
{
    public SensorModel.Generic Model;
}

public class GroundTruthSensorOptions : ISensorOptions
{
    public SensorModel.Generic Model;
}

public interface ISensor
{
    bool IsActive { get; set; }
    SensorType SensorType { get; set; }
    string SensorLink { get; set; }
    string TopicNamespace { get; set; }
    Vector3 Translation { get; set; }
    Vector3 Rotation { get; set; }
    ISensorOptions Options { get; set; }
}

public class Sensor<T> : ISensor where T : ISensorOptions, new()
{
    public bool IsActive { get; set; }
    public SensorType SensorType { get; set; }
    public string SensorLink { get; set; }
    public string TopicNamespace { get; set; }
    public Vector3 Translation { get; set; }
    public Vector3 Rotation { get; set; }
    public T Options { get; set; }

    public Sensor(SensorType sensorType)
    {
        SensorType = sensorType;
        Options = new T();
    }

    // The explicit implementation of ISensor.Options to satisfy the interface contract
    ISensorOptions ISensor.Options
    {
        get => Options;
        set => Options = (T)value;
    }
}

public class SensorSet
{
    public string Name;
    public int Version;
    public List<ISensor> SensorList;

    public SensorSet()
    {
        SensorList = new List<ISensor>();
    }
}

public class GroundTruth
{
    public int car_num;
    public double lat;
    public double lon;
    public double hgt;
    public double vx;
    public double vy;
    public double vz;
    public double yaw;
    public double pitch;
    public double roll;
    public double del_x;
    public double del_y;
    public double del_z;
}

public class Car
{
    public int CarNum;
    public ControlType ControlType;
    public int SpawnPositionIdx;
    public int CameraPriority;
    public CameraMode CameraMode;
    public BodyColor Color;
    public int RosDomain;
}

public class VehSetup
{
    public string Name;
    public int Version;
    // Limited Slip Differential y/n?
    public bool IsLSD;
    // Ideal steering actuator y/n?
    public bool IsIdealSteering;
    // Pure steering acutator delay in s
    public float SteeringDelay;
    // Steering bandwidth in Hz
    public float SteeringBW;
    // Max steering angle in deg
    public float MaxSteeringAngle;
    // Max steering rate in deg/s
    public float MaxSteeringRate;
    // Steering ratio
    public float SteeringRatio;
    // Front roll bar rate in N/m
    public float FrontRollBarRate;
    // Rear roll bar rate in N/m
    public float RearRollBarRate;
    // Brake constant kPa * this = torque at brake rotor
    public float BrakeConstant; 
    // Thermal tyre model on?, ambient temp, track temp (deg C)
    public bool IsThermalTyre;
    public float AmbientTemp;
    public float TrackTemp;
}

public class RaceControl
{
    public int BaseToCarHeartbeat; //1-8
    public byte TrackFlag;
    public byte VehicleFlag;
    public int VehicleRank; 
    public int LapCount; 
    public float LapDistance; 
    public int RoundTargetSpeed;
    public int Laps;
    public float LapTime;
    public float TimeStamp;
}

public class CarCameraPair
{    
    public GameObject Camera { get; set; }
    public GameObject Car { get; set; }

    public CarCameraPair(GameObject camera, GameObject car)
    {
        Camera = camera;
        Car = car;
    }
}

public enum CameraMode
{
    ALONE = 0,
    SPLIT_LEFT = 1,
    SPLIT_RIGHT = 2,
    DISABLE = 3,
}

public enum BodyColor
{
    BLUE = 0,
    NAVY = 1,
    GREEN = 2,
    ORANGE = 3,
    PURPLE = 4,
    RED = 5,
    YELLOW = 6,
    WHITE = 7,
    CARBON = 8,
}

public enum ControlType
{
    HUMAN = 0,
    ROS = 1,
    AI = 2,
}

public enum SensorType
{
    RAPTOR = 0,
    GNSS_INS = 1,
    LIDAR = 2,
    CAMERA = 3,
    RADAR = 4,
    GROUND_TRUTH = 5,
}

public class SensorModel
{
    public enum Generic
    {
        GENERIC,
    }
    public enum GnssIns
    {
        GENERIC_GNSS,
        GENERIC_IMU,
        PWRPAK7,
        VN310,
    }
    public enum Lidar
    {
        GENERIC,
        LUMINAR,
        VELODYNE,
        OUSTER,
        INNOVIZ,
    }
}