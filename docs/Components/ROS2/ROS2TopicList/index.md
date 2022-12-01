
The tables of topics AWSIM publishes and subscribes from can be found below.
The list can be extended by the user for the specific use case.
To see how to custom messages type for ROS2, please refer to [Add custom ROS2 message type](../AddCustomROS2MessageType/index.md) document.

## Publisher list
|category|topic|msg|frame_id|hz|QoS|
|:--|:--|:--|:--|:--|:--|
|clock|`/clock`|`rosgraph_msgs/Clock`|none|`100`|`Best effort`, `Volatile`, `Keep last/1`|
|camera|`/sensing/camera/traffic_light/camera_info`|`sensor_msgs/CameraInfo`|`traffic_light_left_camera/camera_link`|`10`|`Best effort`, `Volatile`, `Keep last/1`|
|camera|`/sensing/camera/traffic_light/image_raw`|`sensor_msgs/Image`|`traffic_light_left_camera/camera_link`|`10`|`Best effort`, `Volatile`, `Keep last/1`|
|gnss|`/sensing/gnss/pose`|`geometry_msgs/Pose`|`gnss_link`|`1`|`Reliable`, `Volatile`, `Keep last/1`|
|gnss|`/sensing/gnss/pose_with_covariance`|`geometry_msgs/PoseWithCovarianceStamped `|`gnss_link`|`1`|`Reliable`, `Volatile`, `Keep last/1`|
|imu|`/sensing/imu/tamagawa/imu_raw`|`sensor_msgs/Imu`|`tamagawa/imu_link`|`30`|`Reliable`, `Volatile`, `Keep last/1000`|
|lidar|`/sensing/lidar/top/pointcloud_raw`|`sensor_msgs/PointCloud2`|`sensor_kit_base_link`|`10`|`Best effort`, `Volatile`, `Keep last/5`|
|lidar|`/sensing/lidar/top/pointcloud_raw_ex`|`sensor_msgs/PointCloud2`|`sensor_kit_base_link`|`10`|`Best effort`, `Volatile`, `Keep last/5`|
|vehicle status|`/vehicle/status/control_mode`|`autoware_auto_vehicle_msgs/ControlModeReport`|none|`30`|`Reliable`, `Volatile`, `Keep last/1`|
|vehicle status|`/vehicle/status/gear_status`|`autoware_auto_vehicle_msgs/GearReport`|none|`30`|`Reliable`, `Volatile`, `Keep last/1`|
|vehicle status|`/vehicle/status/steering_status`|`autoware_auto_vehicle_msgs/SteeringReport`|none|`30`|`Reliable`, `Volatile`, `Keep last/1`|
|vehicle status|`/vehicle/status/turn_indicators_status`|`autoware_auto_vehicle_msgs/TurnIndicatorsReport`|none|`30`|`Reliable`, `Volatile`, `Keep last/1`|
|vehicle status|`/vehicle/status/hazard_lights_status`|`autoware_auto_vehicle_msgs/HazardLightsReport`|none|`30`|`Reliable`, `Volatile`, `Keep last/1`|
|vehicle status|`/vehicle/status/velocity_status`|`autoware_auto_vehicle_msgs/VehicleReport`|none|`30`|`Reliable`, `Volatile`, `Keep last/1`|


## Subscriber list
|category|topic|msg|frame_id|hz|QoS|
|:--|:--|:--|:--|:--|:--|
|control|`/control/command/turn_indicators_cmd`|`autoware_auto_vehicle_msgs/TurnIndicatorsCommand`|none|`10`|`Reliable`,<br> `TransientLocal`,<br> `KeepLast/1`|
|control|`/control/command/hazard_lights_cmd`|`autoware_auto_vehicle_msgs/HazardLightsCommand`|none|`10`|`Reliable`,<br> `TransientLocal`,<br> `KeepLast/1`|
|control|`/control/command/control_cmd`|`autoware_auto_control_msgs/AckermannControlCommand`|none|`60`|`Reliable`,<br> `TransientLocal`,<br> `KeepLast/1`|
|control|`/control/command/gear_cmd`|`autoware_auto_vehicle_msgs/GearCommand`|none|`10`|`Reliable`,<br> `TransientLocal`,<br> `KeepLast/1`|
|control|`/control/command/emergency_cmd`|`tier4_vehicle_msgs/msg/VehicleEmergencyStamped`|none|`60`|`Reliable`,<br> `TransientLocal`,<br> `KeepLast/1`|

