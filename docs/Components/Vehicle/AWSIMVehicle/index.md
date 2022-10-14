# AWSIM Vehicle

!!! info

    AWSIM Vehicle Dynamics is currently under development. However, it is mostly functional.

## Sample Vehicle - Lexus RX450h

<img src=image_0.png width=500px>

AWSIM includes a Lexus RX450h vehicle in the sample. Prefab path is
```
AWSIM\Assets\AWSIM\Prefabs\VehiclesWithSensor\Lexus RX450h 2015 Sample Sensor.prefab
```

## Vehicle Dynamics Concept

The concept is VehicleDynamics suitable for Autoware's `autoware_auto_control_msgs/AckermannControlCommand` and `autoware_auto_vehicle_msgs/GearCommand` msg interface.

features

- Longitudinal control by acceleration (m/s^2)
- Lateral control by two-wheel model
- Yaw, roll and pitch changes by physics engine
- Mass-spring-damer suspension model
- Logical, not mechanical, automatic gears

## Required vehicle parameters

!!! info

    In general, measuring the moment of inertia is not easy, and past papers published by NHTSA are helpful. <br> [Measured Vehicle Inertial Parameters - NHTSA 1998](https://www.researchgate.net/publication/228945609_Measured_Vehicle_Inertial_Parameters-NHTSA)

|parameter|example|unit|
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

## Vehicle Scripts

Vehicle scripts path is `AWSIM\Assets\AWSIM\Scripts\Vehicles\*`

|script|feature|
|:--|:--|
|Vehicle.cs|Vehicle dynamics core class|
|VehicleKeyboardInput.cs|Input control by keyboard|
|VehicleRosInput.cs|Input control by ROS|
|VehicleVisualEffect.cs`|Visual effects such as brake and turn signals|

## Vehicle API
`Vehicle.cs` API list.

### Input API

!!! check

    See also `VehicleKeyboardInput.cs`, `VehicleRosInput.cs`.

|input API|type|feature|
|:--|:--|:--|
|Vehicle.AccelerationInput|float|Acceleration input (m/s^2). In the plane, output the force that will result in this acceleration.<br> On a slope, it is affected by the slope resistance, so it does not match the input.|
|Vehicle.SteerAngleInput|float|Vehicle steering input. Tire angle (degree). Negative is left, positive is right turn tire angle.|
|Vehicle.AutomaticShiftInput|enum|Vehicle gear shift input (AT). PARKING, REVERSE, NEUTRAL, DRIVE.|
|Vehicle.SignalInput|enum|Vehicle turn signal input. NONE, LEFT, RIGHT, HAZARD.|

### Output API

|output API|type|feature|
|:--|:--|:--|
|Vehicle.LocalAcceleration|Vector3|Acceleration(m/s^2) in the local coordinate system of the vehicle.|
|Vehicle.Speed|float|Vehicle speed (m/s).|
|Vehicle.SteerAngle|float|Vehicle steering angle (degree).|
|Vehicle.Signal|enum|Vehicle turn signal.|
|Vehicle.Velocity|vector3|Vehicle velocity (m/s)|
|Vehicle.LocalVelcoity|vector3|Vehicle local velocity (m/s)|
|Vehicle.AngularVelocity|vector3|Vehicle angular velocity (rad/s)|

## ROS2 Control Topics

`VehicleRosInput` subscribes to these topics and applies them to the Vehicle

|topic|msg|frame_id|hz|QoS|
|:--|:--|:--|:--|:--|
|`/control/command/turn_indicators_cmd`|`autoware_auto_vehicle_msgs/TurnIndicatorsCommand`|none|`10`|`Reliable`,<br> `TransientLocal`,<br> `KeepLast/1`|
|`/control/command/hazard_lights_cmd`|`autoware_auto_vehicle_msgs/HazardLightsCommand`|none|`10`|`Reliable`,<br> `TransientLocal`,<br> `KeepLast/1`|
|`/control/command/control_cmd`|`autoware_auto_control_msgs/AckermannControlCommand`|none|`60`|`Reliable`,<br> `TransientLocal`,<br> `KeepLast/1`|
|`/control/command/gear_cmd`|`autoware_auto_vehicle_msgs/GearCommand`|none|`10`|`Reliable`,<br> `TransientLocal`,<br> `KeepLast/1`|
|`/control/command/emergency_cmd`|`tier4_vehicle_msgs/msg/VehicleEmergencyStamped`|none|`60`|`Reliable`,<br> `TransientLocal`,<br> `KeepLast/1`|