# ROS2 topic list

The following is a summary of the *ROS2* topics that the *AWSIM* node subscribes to and publishes on.

!!! Note "Ros2ForUnity"
    *AWSIM* works with *ROS2* thanks to the use of `Ros2ForUnity` - read the details [here](../ROS2ForUnity/).<br>
    If you want to generate a custom message to allow it to be used in *AWSIM* please read this [tutorial](../AddACustomROS2Message/).


## List of subscribers
|                     Category                     | Topic                                  |                     Message type                     | `frame_id` | `Hz`  |                       `QoS`                        |
| :----------------------------------------------: | :------------------------------------- | :--------------------------------------------------: | :--------: | :---: | :------------------------------------------------: |
| <p style="color:rgb(0,255,255);">**Control**</p> |                                        |                                                      |            |       |                                                    |
|                Ackermann Control                 | `/control/command/control_cmd`         | `autoware_auto_control_msgs/AckermannControlCommand` |     -      | `60`  | `Reliable`,<br> `TransientLocal`,<br> `KeepLast/1` |
|                       Gear                       | `/control/command/gear_cmd`            |       `autoware_auto_vehicle_msgs/GearCommand`       |     -      | `10`  | `Reliable`,<br> `TransientLocal`,<br> `KeepLast/1` |
|                 Turn Indicators                  | `/control/command/turn_indicators_cmd` |  `autoware_auto_vehicle_msgs/TurnIndicatorsCommand`  |     -      | `10`  | `Reliable`,<br> `TransientLocal`,<br> `KeepLast/1` |
|                  Hazard Lights                   | `/control/command/hazard_lights_cmd`   |   `autoware_auto_vehicle_msgs/HazardLightsCommand`   |     -      | `10`  | `Reliable`,<br> `TransientLocal`,<br> `KeepLast/1` |
|                    Emergency                     | `/control/command/emergency_cmd`       |   `tier4_vehicle_msgs/msg/VehicleEmergencyStamped`   |     -      | `60`  | `Reliable`,<br> `TransientLocal`,<br> `KeepLast/1` |

## List of publishers

|                        Category                         | Topic                                       |                   Message type                    |               `frame_id`                | `Hz`  |                     `QoS`                      |
| :-----------------------------------------------------: | :------------------------------------------ | :-----------------------------------------------: | :-------------------------------------: | :---: | :--------------------------------------------: |
|      <p style="color:rgb(255,233,127);">**Clock**       | `/clock`                                    |               `rosgraph_msgs/Clock`               |                    -                    | `100` | `Best effort`,<br>`Volatile`,<br>`Keep last/1` |
|    <p style="color:rgb(0,144,255);">**Sensors**</p>     |                                             |                                                   |                                         |       |                                                |
|                         Camera                          | `/sensing/camera/traffic_light/camera_info` |             `sensor_msgs/CameraInfo`              | `traffic_light_left_camera/camera_link` | `10`  | `Best effort`,<br>`Volatile`,<br>`Keep last/1` |
|                         Camera                          | `/sensing/camera/traffic_light/image_raw`   |                `sensor_msgs/Image`                | `traffic_light_left_camera/camera_link` | `10`  | `Best effort`,<br>`Volatile`,<br>`Keep last/1` |
|                          GNSS                           | `/sensing/gnss/pose`                        |               `geometry_msgs/Pose`                |               `gnss_link`               |  `1`  |  `Reliable`,<br>`Volatile`,<br>`Keep last/1`   |
|                          GNSS                           | `/sensing/gnss/pose_with_covariance`        |    `geometry_msgs/PoseWithCovarianceStamped `     |               `gnss_link`               |  `1`  |  `Reliable`,<br>`Volatile`,<br>`Keep last/1`   |
|                           IMU                           | `/sensing/imu/tamagawa/imu_raw`             |                 `sensor_msgs/Imu`                 |           `tamagawa/imu_link`           | `30`  | `Reliable`,<br>`Volatile`,<br>`Keep last/1000` |
|                        Top LiDAR                        | `/sensing/lidar/top/pointcloud_raw`         |             `sensor_msgs/PointCloud2`             |         `sensor_kit_base_link`          | `10`  | `Best effort`,<br>`Volatile`,<br>`Keep last/5` |
|                        Top LiDAR                        | `/sensing/lidar/top/pointcloud_raw_ex`      |             `sensor_msgs/PointCloud2`             |         `sensor_kit_base_link`          | `10`  | `Best effort`,<br>`Volatile`,<br>`Keep last/5` |
| <p style="color:rgb(0,255,144);">**Vehicle Status**</p> |                                             |                                                   |                                         |       |                                                |
|                        Velocity                         | `/vehicle/status/velocity_status`           |    `autoware_auto_vehicle_msgs/VelocityReport`    |               `base_line`               | `30`  |  `Reliable`,<br>`Volatile`,<br>`Keep last/1`   |
|                        Steering                         | `/vehicle/status/steering_status`           |    `autoware_auto_vehicle_msgs/SteeringReport`    |                    -                    | `30`  |  `Reliable`,<br>`Volatile`,<br>`Keep last/1`   |
|                      Control Mode                       | `/vehicle/status/control_mode`              |  `autoware_auto_vehicle_msgs/ControlModeReport`   |                    -                    | `30`  |  `Reliable`,<br>`Volatile`,<br>`Keep last/1`   |
|                          Gear                           | `/vehicle/status/gear_status`               |      `autoware_auto_vehicle_msgs/GearReport`      |                    -                    | `30`  |  `Reliable`,<br>`Volatile`,<br>`Keep last/1`   |
|                     Turn Indicators                     | `/vehicle/status/turn_indicators_status`    | `autoware_auto_vehicle_msgs/TurnIndicatorsReport` |                    -                    | `30`  |  `Reliable`,<br>`Volatile`,<br>`Keep last/1`   |
|                      Hazard Lights                      | `/vehicle/status/hazard_lights_status`      |  `autoware_auto_vehicle_msgs/HazardLightsReport`  |                    -                    | `30`  |  `Reliable`,<br>`Volatile`,<br>`Keep last/1`   |
| <p style="color:rgb(144,255,144);">**Ground Truth**</p> |                                             |                                                   |                                         |       |                                                |
|                          Pose                           | `/awsim/ground_truth/vehicle/pose`          |            `geometry_msgs/PoseStamped`            |               `base_link`               | `100` |  `Reliable`,<br>`Volatile`,<br>`Keep last/1`   |

