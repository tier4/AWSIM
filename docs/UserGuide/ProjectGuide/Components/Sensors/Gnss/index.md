
## GNSS Sensor

GNSS senor is a component which simulates the position of vehicle computed by the Global Navigation Satellite System. The GNSS sensor outputs the position in the MGRS coordinate system.

### Prefabs

The component prefab can be found under the following path: `Assets\AWSIM\Prefabs\Sensors\GnssSensor.prefab`

### Scripts

All the most important scripts can be found under the ``Assets\AWSIM\Prefabs\Sensors\Gnss\*` path.

The table below describes features contained in each provided script:

| script               | feature                                                                                                                                                             |
| :------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| GnssSensor.cs        | Core GNSS sensor. Publishes pose and poseWithCovarianceStamped in MGRS coordinate system. Requires MgrsReference of the Environment for the output data conversion. |
| GnssRos2Publisher.cs | Converts the data output from GnssSensor to ROS2 message and publishes it.                                                                                          |

### Output Data

The following table describes `GnssSensor.OutputData` properties:

| field        | type    | feature                                 |
| :----------- | :------ | :-------------------------------------- |
| MgrsPosition | Vector3 | Position in the MGRS coordinate system. |

### Published Topics

The data output is published to the following topics:

| topic                                | msg                                        | frame_id    | hz   | QoS                                   |
| :----------------------------------- | :----------------------------------------- | :---------- | :--- | :------------------------------------ |
| `/sensing/gnss/pose`                 | `geometry_msgs/Pose`                       | `gnss_link` | `1`  | `Reliable`, `Volatile`, `Keep last/1` |
| `/sensing/gnss/pose_with_covariance` | `geometry_msgs/PoseWithCovarianceStamped ` | `gnss_link` | `1`  | `Reliable`, `Volatile`, `Keep last/1` |

**CameraSensor [10% current]**
<!-- TODO copied old, needs to be adjusted a lot (10%) -->

(prefab location, purpose of existence, link, **screen**)

- Camera component (https://docs.unity3d.com/Manual/class-Camera.html)
- Camera Sensor Script (parameters, output, gui - **screen**)
    - Distortion shader (what is it and where is it located)
    - ROS image shader (what is it and where is it located)
- Camera Ros Publisher Script (topics, frame_id, qos)
- Impact on the traffic lights recognition in autoware
