# AWSIM Vehicle

!!! info

    AWSIM Vehicle Dynamics is mostly functional, but still under development.

## Sample Vehicle - Lexus RX450h

<img src=image_0.png width=500px>

AWSIM project includes a Lexus RX450h vehicle model. Prefab path is
```
AWSIM\Assets\AWSIM\Prefabs\VehiclesWithSensor\Lexus RX450h 2015 Sample Sensor.prefab
```

## Vehicle Dynamics Concept

The concept for VehicleDynamics is suitable for Autoware's `autoware_auto_control_msgs/AckermannControlCommand` and `autoware_auto_vehicle_msgs/GearCommand` message interface usage.

Supported features:

- Longitudinal control by acceleration (m/s^2)
- Lateral control by two-wheel model
- Yaw, roll and pitch controlled by physics engine
- Mass-spring-damper suspension model
- Logical, not mechanical, automatic gears change

## Required vehicle parameters

!!! info

    In general, measuring the moment of inertia is not easy, and past papers published by NHTSA are helpful. <br> [Measured Vehicle Inertial Parameters - NHTSA 1998](https://www.researchgate.net/publication/228945609_Measured_Vehicle_Inertial_Parameters-NHTSA)

|Parameter|Value|unit|
|:--|:--|:--|
|mass|1500|kg|
|wheel base|2.5|m|
|tread width|Ft : 1.8, Rr : 1.8|m|
|center of mass position|x : 0, y : 0.5, z : 0|m|
|moment of inertia|yaw : 2000, roll : 2000, pitch : 700|kgm^2|
|spring rate|Ft : 55000, Rr : 48000|N|
|damper rate|Ft : 3000, Rr : 2500|N/s|
|suspension stroke|Ft : 0.2, Rr 0.2|m|
|wheel radius|0.365|m|

## Vehicle control scripts

Scripts that simulate and support the car's behavior are placed under `AWSIM/Assets/AWSIM/Scripts/Vehicles` directory.


|script|feature|
|:--|:--|
|Vehicle.cs|Vehicle dynamics core class|
|VehicleKeyboardInput.cs|Input control by keyboard|
|VehicleRosInput.cs|Input control by ROS|
|VehicleVisualEffect.cs`|Visual effects (e.g. brake and turn signals)|

## Vehicle API

The following section describes the API of `Vehicle.cs` script.

### Input API

!!! check

    See also `VehicleKeyboardInput.cs`, `VehicleRosInput.cs`.

|input API|type|feature|
|:--|:--|:--|
|Vehicle.AccelerationInput|float|Acceleration input (m/s^2). On the plane, output the force that will result in this acceleration.<br> On a slope, it is affected by the slope resistance, so it does not match the input.|
|Vehicle.SteerAngleInput|float|Vehicle steering input (degree). Negative steers left, positive right|
|Vehicle.AutomaticShiftInput|enumeration|Vehicle gear shift input (AT).<br>Values: PARKING, REVERSE, NEUTRAL, DRIVE.|
|Vehicle.SignalInput|enumeration|Vehicle turn signal input. <br>Values: NONE, LEFT, RIGHT, HAZARD.|

### Output API

|output API|type|feature|
|:--|:--|:--|
|Vehicle.LocalAcceleration|Vector3|Acceleration(m/s^2) in the local coordinate system of the vehicle.|
|Vehicle.Speed|float|Vehicle speed (m/s).|
|Vehicle.SteerAngle|float|Vehicle steering angle (degree).|
|Vehicle.Signal|enumeration|Vehicle turn signal.|
|Vehicle.Velocity|Vector3|Vehicle velocity (m/s)|
|Vehicle.LocalVelcoity|Vector3|Vehicle local velocity (m/s)|
|Vehicle.AngularVelocity|Vector3|Vehicle angular velocity (rad/s)|

## ROS2 Control Topics

`VehicleRosInput` subscribes to these topics and applies them to the Vehicle

| topic                                  |msg|frame_id|hz|QoS|
|:---------------------------------------|:--|:--|:--|:--|
| `/control/command/turn_indicators_cmd` |`autoware_auto_vehicle_msgs/TurnIndicatorsCommand`|none|`10`|`Reliable`,<br> `TransientLocal`,<br> `KeepLast/1`|
| `/control/command/hazard_lights_cmd`   |`autoware_auto_vehicle_msgs/HazardLightsCommand`|none|`10`|`Reliable`,<br> `TransientLocal`,<br> `KeepLast/1`|
| `/control/command/control_cmd_auto`    |`autoware_auto_control_msgs/AckermannControlCommand`|none|`60`|`Reliable`,<br> `TransientLocal`,<br> `KeepLast/1`|
| `/control/command/gear_cmd`            |`autoware_auto_vehicle_msgs/GearCommand`|none|`10`|`Reliable`,<br> `TransientLocal`,<br> `KeepLast/1`|
| `/control/command/emergency_cmd`       |`tier4_vehicle_msgs/msg/VehicleEmergencyStamped`|none|`60`|`Reliable`,<br> `TransientLocal`,<br> `KeepLast/1`|