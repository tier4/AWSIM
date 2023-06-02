**IMUSensor [50% current]**
<!-- TODO copied old, needs to be adjusted (50%) -->

(prefab location, purpose of existence, link, **screen**)

- Imu Sensor Script (gravity, output)
- Imu Ros Publisher Script (topics, frame_id, qos)

# IMU Sensor

The document describes inertial measurement unit sensor simulation component.

## Prefabs
Path : `Assets\AWSIM\Prefabs\Sensors\IMUSensor.prefab`

## Scripts

All the most important scripts can be found under the `Assets\AWSIM\Scripts\Sensors\Imu\*`

The table below describes features contained in each provided script:

| script              | feature                                                                                                                                                                                |
| :------------------ | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ImuSensor.cs        | Core Inertial Measurement Unit Sensor.<br>Measures the Acceleration(m/s^2) and AngularVelocity(rad/s) based on the Transform of the GameObject to which this component is attached to. |
| ImuRos2Publisher.cs | Converts the data output from ImuSensor-specific struct to ROS2 message and publishes it.                                                                                              |

## Output Data

The following table describes `ImuSensor.OutputData` properties:

| field              | type    | feature                           |
| :----------------- | :------ | :-------------------------------- |
| LinearAcceleration | Vector3 | Measured acceleration (m/s^2)     |
| AungularVelocity   | Vector3 | Measured angular velocity (rad/s) |

## Published Topics

The data output is published to the following topics:

| topic                           | msg               | frame_id            | hz   | QoS                                      |
| :------------------------------ | :---------------- | :------------------ | :--- | :--------------------------------------- |
| `/sensing/imu/tamagawa/imu_raw` | `sensor_msgs/Imu` | `tamagawa/imu_link` | `30` | `Reliable`, `Volatile`, `Keep last/1000` |
