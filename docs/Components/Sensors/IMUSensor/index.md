# IMU Sensor

IMU Sensor is IMU Sensor stands for Inetial Measurement Unit Sensor.  

## Prefabs
Path : `Assets\AWSIM\Prefabs\Sensors\IMUSensor.prefab`

## Scripts

Path : `Assets\AWSIM\Scripts\Sensors\Imu\*`

|script|feature|
|:--|:--|
|ImuSensor.cs|IMU (Inertial Measurement Unit) Sensor.<br>Measure the Acceleration(m/s^2) and AngularVelocity(rad/s) based on the Transform of the GameObject to which this component is attached, and publish it to ROS2.|
|ImuRos2Publisher.cs|Convert the data output from ImuSensor to ROS2 msg and Publish.|

## Output Data
`ImuSensor.OutputData` properties

|field|type|feature|
|:--|:--|:--|
|LinearAcceleration|Vector3|Measured acceleration (m/s^2)|
|AungularVelocity|Vector3|Measured angular velocity (rad/s)|

## ROS2 Publish Topics

Topics published by `ImuRos2Publisher`

|topic|msg|frame_id|hz|QoS|
|:--|:--|:--|:--|:--|
|`/sensing/imu/tamagawa/imu_raw`|`sensor_msgs/Imu`|`tamagawa/imu_link`|`30`|`Reliable`, `Volatile`, `Keep last/1000`|