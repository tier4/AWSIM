# GNSS Sensor

GNSS Sensor is Global Navigation Satellite System Sensor. The GNSS sensor outputs the position in the MGRS coordinate system.

## Prefabs

Path : `Assets\AWSIM\Prefabs\Sensors\GnssSensor.prefab`

## Scripts

Path : `Assets\AWSIM\Prefabs\Sensors\Gnss\*`

|script|feature|
|:--|:--|
|GnssSensor.cs|GNSS sensor. Publish pose and poseWithCovarianceStamped in MGRS coordinate system. Need to set the MgrsReference of the Environment for the MGRS coordinate system.|
|GnssRos2Publisher.cs|Convert the data output from GnssSensor to ROS2 msg and Publish.|

## Output Data
`GnssSensor.OutputData` properties

|field|type|feature|
|:--|:--|:--|
|MgrsPosition|Vector3|Position in the MGRS coordinate system.|

## ROS2 Publish Topics

Topics published by `GnssRos2Publisher`

|topic|msg|frame_id|hz|QoS|
|:--|:--|:--|:--|:--|
|`/sensing/gnss/pose`|`geometry_msgs/Pose`|`gnss_link`|`1`|`Reliable`, `Volatile`, `Keep last/1`|
|`/sensing/gnss/pose_with_covariance`|`geometry_msgs/PoseWithCovarianceStamped `|`gnss_link`|`1`|`Reliable`, `Volatile`, `Keep last/1`|