# Abstract and installation
Feature to operate AWSIM environment from the Rviz plugins.

<a href="../NpcSpawner/npc_spawner.png" data-lightbox="AwsimRvizPlugins" data-title="AwsimRvizPlugins" data-alt="AwsimRvizPlugins"><img src="../NpcSpawner/npc_spawner.png"></a>

The following features are implemented:

- [2D Pose Teleport](../2dPoseTeleport/index.md): Teleport AWSIM EGO pose from Rviz GUI tool
- [Npc Spawner](../NpcSpawner/index.md): Spawn AWSIM Npc using Rviz GUI tool

Rviz plugins are implemented in the following repositories:

- [https://github.com/tier4/AwsimRvizPlugins](https://github.com/tier4/AwsimRvizPlugins)

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
