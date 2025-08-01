`IMUSensor` is a component that simulates an *IMU* (*Inertial Measurement Unit*) sensor.
Measures acceleration (${m}/{s^2}$) and angular velocity (${rad}/{s}$) based on the transformation of the *GameObject* to which this component is attached.

## Prefab

```{.yml .no-copy}
Assets/Awsim/Entity/EgoVehicle/Imu/ImuSensor.prefab
```

<br>

## ImuSensor class

`ImuSensor` outputs linear acceleration and angular velocity at a configured period.

### How to use

1. Initialize `ImuSensor`.
1. Other MonoBehaviour's`FixedUpdate()` calls `ImuSensor.OnFixedUpdate()`


### Setting parameters

|Type|Parameter|Feature|
|:--|:--|:--|
|`int`|`_outputHz`|Period to output.|
|`bool`|`_enableGravity`|Flag to include gravity in output.|

### Output data

It is contained in the `ImuSensor.IReadOnlyOutputData` type.

|Type|Parameter|Feature|
|:--|:--|:--|
|`Vector3`|`LinearAcceleration`|Acceleration (${m}/{s^2}$).|
|`Vector3`|`AngularVelocity`|Angular velocity (${rad}/{s}$).|

### Add output callback

`ImuSensor` can add an `Action` type callback that takes `ImuSensor.IReadOnlyOutputData` as an argument.

```cs
// sample code from ImuRos2Publisher.cs
_gnssSensor.OnOutput += Publish;
```

<br >

## ImuRos2Publisher class

`ImuRos2Publisher` converts the output of `ImuSensor` to ROS2 and publishes the topic.

### Setting parameters

|Type|Parameter|Feature|
|:--|:--|:--|
|`string`|`_topic`|`geometry_msgs/Pose` msg topic name.|
|`string`|`_frameID`|Frame ID of ros2.|
|`QosSettings`|`_qosSettings`|Quality of Service settings of ros2.|
|`ImuSensor`|`_imuSensor`|Target `ImuSensor` instance.|

### Default publish topics

`ImuRos2Publisher` is configured by default with the following two topics publsiih.

| Topic                                | Message type                                                                                                                   | `frame_id`  | `Hz`  | `QoS`                                                            |
|--:-----------------------------------|--:-----------------------------------------------------------------------------------------------------------------------------|--:----------|--:-:--|--:---------------------------------------------------------------|
| `/sensing/imu/tamagawa/imu_raw`             | [`sensor_msgs/Imu`](https://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)                                                                                   | `tamagawa/imu_link`                     | `30`  | <ul><li>`Reliable`</li><li>`Volatile`</li><li>`Keep last/1000`</li> |