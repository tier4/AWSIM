<!-- TODO everything -->
<!-- DM: nie da sie tego podzielic na podsekcje? duzo tutaj tego jest -->
- Ros2Unity (description, reasons for using, hyperlink)
    - Default message types (short description with location)
    - List of used topics
    - Add custom ROS2 message type [70% Current] (**screens**)
    - Accessing and filling in message fields (short example - including array)
- RobotecGPULidar (description, reasons for using, hyperlink)
    - SceneManager Script (description)

## ROS 2 For Unity

AWSIM uses [Ros2ForUnity](https://github.com/RobotecAI/ros2-for-unity) module for ROS 2 communication. `ROS 2 For Unity` is a high-performance communication solution to connect Unity3D and ROS 2 ecosystem in a ROS 2 "native" way.

This document briefly guides you on how to add your custom messages to the simulation. For a detailed description, please see [ROS 2 For Unity Readme](https://github.com/RobotecAI/ros2-for-unity/blob/master/README.md).

### Prerequisites

`ROS 2 For Unity` depends on a [ros2cs](https://github.com/RobotecAI/ros2cs) - a C## .NET library for ROS 2. You don't have to install it manually, but there are a few prerequisites that must be resolved first.

Please refer to the following prerequisites section for your OS:

=== "Ubuntu"
    [Ubuntu prerequisites](https://github.com/RobotecAI/ros2cs/blob/master/README-UBUNTU.md#prerequisites)

=== "Windows"
    [Windows prerequisites](https://github.com/RobotecAI/ros2cs/blob/1.2.0/README-WINDOWS.md#prerequisites)

Additional required tools:

- Git
- Shell:

    === "Ubuntu"
        `bash`

    === "Windows"
        `powershell`

### Default message types
<!-- TODO short description with location -->

### List of used topics

The tables of topics AWSIM publishes and subscribes from can be found below.
The list can be extended by the user for the specific use case.
To see how to custom messages type for ROS 2, please refer to [Add custom ROS 2 message type](#add-custom-ros-2-message-type) document.

#### Publisher list
| category       | topic                                       | msg                                               | frame_id                                | hz    | QoS                                      |
| :------------- | :------------------------------------------ | :------------------------------------------------ | :-------------------------------------- | :---- | :--------------------------------------- |
| clock          | `/clock`                                    | `rosgraph_msgs/Clock`                             | none                                    | `100` | `Best effort`, `Volatile`, `Keep last/1` |
| camera         | `/sensing/camera/traffic_light/camera_info` | `sensor_msgs/CameraInfo`                          | `traffic_light_left_camera/camera_link` | `10`  | `Best effort`, `Volatile`, `Keep last/1` |
| camera         | `/sensing/camera/traffic_light/image_raw`   | `sensor_msgs/Image`                               | `traffic_light_left_camera/camera_link` | `10`  | `Best effort`, `Volatile`, `Keep last/1` |
| gnss           | `/sensing/gnss/pose`                        | `geometry_msgs/Pose`                              | `gnss_link`                             | `1`   | `Reliable`, `Volatile`, `Keep last/1`    |
| gnss           | `/sensing/gnss/pose_with_covariance`        | `geometry_msgs/PoseWithCovarianceStamped `        | `gnss_link`                             | `1`   | `Reliable`, `Volatile`, `Keep last/1`    |
| imu            | `/sensing/imu/tamagawa/imu_raw`             | `sensor_msgs/Imu`                                 | `tamagawa/imu_link`                     | `30`  | `Reliable`, `Volatile`, `Keep last/1000` |
| lidar          | `/sensing/lidar/top/pointcloud_raw`         | `sensor_msgs/PointCloud2`                         | `sensor_kit_base_link`                  | `10`  | `Best effort`, `Volatile`, `Keep last/5` |
| lidar          | `/sensing/lidar/top/pointcloud_raw_ex`      | `sensor_msgs/PointCloud2`                         | `sensor_kit_base_link`                  | `10`  | `Best effort`, `Volatile`, `Keep last/5` |
| vehicle status | `/vehicle/status/control_mode`              | `autoware_auto_vehicle_msgs/ControlModeReport`    | none                                    | `30`  | `Reliable`, `Volatile`, `Keep last/1`    |
| vehicle status | `/vehicle/status/gear_status`               | `autoware_auto_vehicle_msgs/GearReport`           | none                                    | `30`  | `Reliable`, `Volatile`, `Keep last/1`    |
| vehicle status | `/vehicle/status/steering_status`           | `autoware_auto_vehicle_msgs/SteeringReport`       | none                                    | `30`  | `Reliable`, `Volatile`, `Keep last/1`    |
| vehicle status | `/vehicle/status/turn_indicators_status`    | `autoware_auto_vehicle_msgs/TurnIndicatorsReport` | none                                    | `30`  | `Reliable`, `Volatile`, `Keep last/1`    |
| vehicle status | `/vehicle/status/hazard_lights_status`      | `autoware_auto_vehicle_msgs/HazardLightsReport`   | none                                    | `30`  | `Reliable`, `Volatile`, `Keep last/1`    |
| vehicle status | `/vehicle/status/velocity_status`           | `autoware_auto_vehicle_msgs/VehicleReport`        | none                                    | `30`  | `Reliable`, `Volatile`, `Keep last/1`    |


#### Subscriber list
| category | topic                                  | msg                                                  | frame_id | hz   | QoS                                                |
| :------- | :------------------------------------- | :--------------------------------------------------- | :------- | :--- | :------------------------------------------------- |
| control  | `/control/command/turn_indicators_cmd` | `autoware_auto_vehicle_msgs/TurnIndicatorsCommand`   | none     | `10` | `Reliable`,<br> `TransientLocal`,<br> `KeepLast/1` |
| control  | `/control/command/hazard_lights_cmd`   | `autoware_auto_vehicle_msgs/HazardLightsCommand`     | none     | `10` | `Reliable`,<br> `TransientLocal`,<br> `KeepLast/1` |
| control  | `/control/command/control_cmd`         | `autoware_auto_control_msgs/AckermannControlCommand` | none     | `60` | `Reliable`,<br> `TransientLocal`,<br> `KeepLast/1` |
| control  | `/control/command/gear_cmd`            | `autoware_auto_vehicle_msgs/GearCommand`             | none     | `10` | `Reliable`,<br> `TransientLocal`,<br> `KeepLast/1` |
| control  | `/control/command/emergency_cmd`       | `tier4_vehicle_msgs/msg/VehicleEmergencyStamped`     | none     | `60` | `Reliable`,<br> `TransientLocal`,<br> `KeepLast/1` |


### Add custom ROS 2 message type
<!-- TODO needs to be adjusted (70%), this is copied old -->

#### Assumptions

To include a custom ROS 2 message, you have to build `ROS 2 For Unity` with your custom message package. The following steps will assume that:

- the temporary `ROS 2 For Unity` build folder is located in the home directory

    === "Ubuntu"
        `~/`

    === "Windows"
        `C:\`

- your message package is called `custom_msgs` and is located under the path:

    === "Ubuntu"
        `~/custom_msgs`

    === "Windows"
        `C:\custom_msgs`
- ROS 2 version is `humble` (`foxy` and `galactic` are also supported, please check the current AWSIM version for more information about its ROS2 version) and is located in

    === "Ubuntu"
        `/opt/ros/humble`

    === "Windows"
        `C:\ros2_humble`

#### Workspace preparation

1. Clone `ROS 2 For Unity` repository.

    ```
    git clone https://github.com/RobotecAI/ros2-for-unity
    ```

1. Pull dependent repositories

    === "Ubuntu"
        ```bash
        cd ~/ros2-for-unity
        . /opt/ros/humble/setup.bash
        ./pull_repositories.sh
        ```

    === "Windows"
        ```powershell
        cd C:\ros2-for-unity
        C:\ros2_humble\local_setup.ps1
        .\pull_repositories.ps1
        ```

#### Setup `custom_msgs` package

##### Package hosted on git repository

1. Open `ros2-for-unity/ros2_for_unity_custom_messages.repos` file in your favorite editor,
2. Modify the file contents, so it points to `custom_msgs` repository by changing `<REPOSITORY_URL>` to repository address, `<BRANCH_NAME>` to desired branch and `<PACKAGE_NAME>` to your local package name representation - in this case `custom_msgs`. The configured file should look as follows:

    ```
    ## NOTE: Use this file if you want to build with custom messages that reside in a separate remote repo.
    ## NOTE: use the following format

    repositories:
        src/ros2cs/custom_messages/<PACKAGE_NAME>:
            type: git
            url: <REPOSITORY_URL>
            version: <BRANCH_NAME>
    ```
3. Now please, pull repositories to download and setup the package:
    
    === "Ubuntu"
        ```bash
        ./pull_repositories.sh
        ```

    === "Windows"
        ```powershell
        .\pull_repositories.ps1
        ```

##### Package contained on local machine

1. Simply `custom_msgs` package to `src/ros2cs/custom_messages` directory
    
    === "Ubuntu"
        ```bash
        cp -r ~/custom_msgs ~/ros2-for-unity/src/ros2cs/custom_messages/
        ```

    === "Windows"
        ```bash
        cp -r ~/custom_msgs ~/ros2-for-unity/src/ros2cs/custom_messages/
        ```

#### Build ROS 2 For Unity

Build `ROS 2 For Unity` using the following command:

=== "Ubuntu"
    ```
    ./build.sh --standalone
    ```

=== "Windows"
    ```
    .\build.ps1 -standalone
    ```

#### Install `custom_msgs` to AWSIM

New `ROS 2 For Unity` build, which you just made, contains multiple libraries that already exist in the AWSIM. To install `custom_msgs` and not copy all other unnecessary files, you should get the `custom_msgs` related libraries only and copy them to the analogous directories in `AWSIM/Assets/Ros2ForUnity`.

You can find them in following directories:

- `ros2-for-unity/install/asset/Ros2ForUnity/Plugins` which names matches `custom_msgs_*`

=== "Ubuntu"
    ```
    `ros2-for-unity/install/asset/Ros2ForUnity/Plugins/Linux/x86_64/` which names matches `libcustom_msgs_*`
    ```

=== "Windows"
    ```
    `ros2-for-unity/install/asset/Ros2ForUnity/Plugins/Windows/x86_64/` which names matches `custom_msgs_*`
    ```

To automate the process, you can use these commands (change `<AWSIM_DIR>` to your E2Simulator path and `<CUSTOM_MSGS_PACKAGE_NAME>` to a custom messages package):

=== "Ubuntu"
    ```
    find ~/ros2-for-unity/install/asset/Ros2ForUnity/Plugins -maxdepth 1 -name "<CUSTOM_MSGS_PACKAGE_NAME>*"    -type f -exec cp {} <AWSIM_DIR>/Assets/Ros2ForUnity/Plugins \;
    find ~/ros2-for-unity/install/asset/Ros2ForUnity/Plugins/Linux/x86_64 -maxdepth 1 -name     "lib<CUSTOM_MSGS_PACKAGE_NAME>*" -type f -exec cp {} <AWSIM_DIR>/Assets/Ros2ForUnity/Plugins/Linux/x86_64 \;
    ```

=== "Windows"
    ```
    Get-ChildItem C:\ros2-for-unity\install\asset\Ros2ForUnity\Plugins\* -Include @('<CUSTOM_MSGS_PACKAGE_NAME>*') | Copy-Item -Destination <AWSIM_DIR>\Assets\Ros2ForUnity\Plugins
    Get-ChildItem C:\ros2-for-unity\install\asset\Ros2ForUnity\Plugins\Windows\x86_64\* -Include @('<CUSTOM_MSGS_PACKAGE_NAME>*') | Copy-Item -Destination <AWSIM_DIR>\Assets\Ros2ForUnity\Plugins\Windows\x86_64
    ```

### Accessing and filling in message fields
<!-- TODO (short example - including array) -->

## RobotecGPULidar
<!-- TODO description, reasons for using, hyperlink -->
[Robotec GPU Lidar](https://github.com/RobotecAI/RobotecGPULidar) is an open source high performance lidar simulator running on CUDA-enabled GPUs.
It is used because of the performance benefits (calculations happen on the GPU).

RobotecGPULidar can also perform parallelized point cloud calculations e.g. transformations.

### SceneManager Script
<!-- TODO description -->
Scene Manager is a script responsible for synchronizing model data between Unity and RGL.

SceneManager synchronizes 3D model data in time keeping track of what is changing between frames.
It obtains 3D models from Game Objects when they are needed and deletes them when they are no longer needed.

This script can obtain 3D model in several different ways from a Game Object:

1. From active colliders in the Game Object
2. From mesh in the Game Object
3. From mesh for non-skinned MeshRenderers and from colliders for SkinnedMeshRenderers
