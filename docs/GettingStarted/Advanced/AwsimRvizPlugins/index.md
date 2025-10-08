# AwsimRvizPlugins
Feature to operate AWSIM environment from the Rviz plugins.

<popup-img src="./npc_spawner.png" alt="AwsimRvizPlugins"></popup-img>

The following features are implemented:

- `2D Pose Teleport`: Teleport AWSIM EGO pose from Rviz GUI tool
- `Npc Spawner`: Spawn AWSIM Npc using Rviz GUI tool

Rviz plugins are implemented in the following repositories:

- [https://github.com/tier4/AwsimRvizPlugins](https://github.com/tier4/AwsimRvizPlugins)

!!!info
    If you want to setup `Unity` scene with `AwsimRvizPlugins`, see [here](../../../DeveloperGuide/Layer/Usecase/AwsimRvizPlugins/index.md).

## Installation
### Install only this plugins to Rviz
1. clone this repository
```
git clone git@github.com:tier4/AwsimRvizPlugins.git
```
2. build package
```
cd AwsimRvizPlugins
source /opt/ros/humble/setup.bash
colcon build
```
3. source package
You must `source` each time you launch a terminal.
```
source install/setup.sh
```
4. launch Rviz application and use tools
```
# rviz2
# ros2 launch ...
```

### Install with Autoware (pilot-auto)
1. clone Autoware repository
2. add description of AwsimRvizPlugins to autoware.repos
```
repositories:
  # please add the following statement to autoware.repos to use AwsimRvizPlugins
  simulator/awsim_rviz_plugins:
    type: git
    url: git@github.com:tier4/AwsimRvizPlugins.git
```
3. Introduce `Autoware (pilot-auto)` according to [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/).

## 2D Pose Teleport
Teleport AWSIM EGO using Rviz GUI tool.
<popup-img src="./2d_pose_teleport.png" alt="2D Pose Teleport"></popup-img>

### How to use
1. On Rviz, Click the plus button on the toolbar and select `awsim_rviz_plugins/2dPoseTeleport` from the list.
<popup-img src="./tool_bar_ego.png" alt="2D Pose Teleport Setup"></popup-img>
2. On Rviz, Click on `2D Pose Teleport` button from the toolbar and select it.
3. On AWSIM, Play binary or Play scene.
4. On the map displayed in Rviz, drag the cursor to the location and orientation where you want to move the EGO.

If `Autoware (pilot-auto)` is running with, press the `Initialize with GNSS` button to perform localilization again.

## Nps Spawner
Spawn AWSIM Npc using Rviz GUI tool.<br>
Type and velocity of spawned Npc is specified from Rviz display.

<popup-img src="./npc_spawner.png" alt="Npc Spawner"></popup-img>

### How to use
1. On Rviz, Click the plus button on the toolbar and select `awsim_rviz_plugins/NpcSpawner` from the list.
<popup-img src="./tool_bar_npc.png" alt="Tool Bar"></popup-img>
2. On Rviz, Click on `Npc Spawner` button from the toolbar and select it.
3. On AWSIM, Play binary or Play scene.
4. On the map displayed in Rviz, drag the cursor to the location and orientation where you want to spawn the Npc.

If you want to change type and velocity of spawned Npc, do the following:

1. On Rviz, Click the `Add` button on the `Display` panel and select `awsim_rviz_plugins/NpcSpawnerStatus` from the list.
<popup-img src="./status_panel.png" alt="Status Panel"></popup-img>
2. On Rviz, Change the value of `Npc Type` and `Velocity [km/h]`.
