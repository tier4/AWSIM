`GnssSensor` is a component which simulates the position of vehicle computed by the *Global Navigation Satellite System* based on the transformation of the *GameObject* to which this component is attached.
The `GnssSensor` outputs the position in the [*MGRS*](https://www.maptools.com/tutorials/mgrs/quick_guide) coordinate system and [*Geo coordinate system*](https://en.wikipedia.org/wiki/Geographic_coordinate_system).

## Prefab

```{.yml .no-copy}
Assets/Awsim/Entity/EgoVehicle/Gnss/GnssSensor.prefab
```

<br>


## GnssSensor class

`GnssSensor` outputs MGRS and GeoCoordinate coordinate positions based on the configured period.

### prerequisites

`MgrsPosition` and `GeoCoordinatePosition` need to be set up. From these classes, the output is converted to each coordinate system by considering the Unity world coordinate system origin and sensor position.

### How to use

1. Setup `MgrsPosition` and `GeoCoordinatePosition` int the scene.
1. Initialize `GnssSensor`.


### Setting parameters

|Type|Parameter|Feature|
|:--|:--|:--|
|`int`|`_outputHz`|Period to output.|

### Output data

It is contained in the `GnssSensor.IReadOnlyOutputData` type.

|Type|Parameter|Feature|
|:--|:--|:--|
|`Mgrs`|`Mgrs`|MGRS coordinate system position.|
|`GeoCoordinate`|`GeoCoordinate`|GeoCoordinate coordinate system position.|

### Geo reference

`GnssSensor` outputs the sum of the origin value set for each geo reference and the value obtained by converting the Unity world coordinates to each geo reference. There are two classes of georeferencing.

|Geo reference|Feature|
|:--|:--|
|`MgrsPosition`|Origin value of the Unity world coordinate system (0, 0, 0) transformed in the MGRS system.|
|`GeoCoordinatePosition`|Origin value of the Unity world coordinate system (0, 0, 0) transformed in the Geo coordinate system.|




### Add output callback

GnssSensor can add an `Action` type callback that takes `GnssSensor.IReadOnlyOutputData` as an argument.

```cs
// sample code from GnssRos2Publisher.cs
_gnssSensor.OnOutput += Publish;
```

<br> 

## GnssRos2Publisher class

`GnssRos2Publisher` converts the output of `GnssSensor` to ROS2 and publishes the topic.

### Setting parameters

|Type|Parameter|Feature|
|:--|:--|:--|
|`string`|`_poseTopic`|`geometry_msgs/Pose` msg topic name.|
|`string`|`_poseWithCovarianceStampedTopic`|`geometry_msgs/PoseWithCovarianceStamped` msg topic name.|
|`string`|`_frameID`|Frame ID of ros2.|
|`QosSettings`|`_qosSettings`|Quality of Service settings of ros2.|
|`GnssSensor`|`_gnssSensor`|Target `GnssSensor` instance.|

### Default publish topics

`GnssRos2Publisher` is configured by default with the following two topics publsiih.

| Topic                                | Message type                                                                                                                   | `frame_id`  | `Hz`  | `QoS`                                                            |
|--:-----------------------------------|--:-----------------------------------------------------------------------------------------------------------------------------|--:----------|--:-:--|--:---------------------------------------------------------------|
| `/sensing/gnss/pose`                 | [`geometry_msgs/Pose`](https://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html)                                           | `gnss_link` | `1`   | <ul><li>`Reliable`</li><li>`Volatile`</li><li>`Keep last/1`</li> |
| `/sensing/gnss/pose_with_covariance` | [`geometry_msgs/PoseWithCovarianceStamped`](https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html) | `gnss_link` | `1`   | <ul><li>`Reliable`</li><li>`Volatile`</li><li>`Keep last/1`</li> |
