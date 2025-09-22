# AwsimRvizPlugins
Feature to operate AWSIM environment from the Rviz plugins.

<a href="./npc_spawner.png" data-lightbox="AwsimRvizPlugins" data-title="" data-alt="AwsimRvizPlugins"><img src="./npc_spawner.png"></a>

The following features are implemented:

- `2D Pose Teleport`: Teleport AWSIM EGO pose from Rviz GUI tool
- `Npc Spawner`: Spawn AWSIM Npc using Rviz GUI tool

Rviz plugins are implemented in the following repositories:

- [https://github.com/tier4/AwsimRvizPlugins](https://github.com/tier4/AwsimRvizPlugins)

!!!info
    If you want to **use** `AwsimRvizPlugins` with `AWSIM`, see [here](../../../../GettingStarted/Advanced/AwsimRvizPlugins/index.md).

## Abstract

### 2D Pose Teleport
Teleport AWSIM EGO using Rviz GUI tool.
<a href="./2d_pose_teleport.png" data-lightbox="2D Pose Teleport" data-title="" data-alt="2D Pose Teleport"><img src="./2d_pose_teleport.png"></a>

`awsim_rviz_plugins/2dPoseTeleport` tool gets position and orientation by dragging on the map displayed in Rviz, and publishes those as a `/awsim/awsim_rviz_plugins/pose_teleport/pose_with_covariance` topic.

AWSIM subscribes this topic and updates the coordinates of the EGO.

y-axis of destination position of teleport is calculated using ray-casting.  
The highest object at the specified x-z coordinates is considered the ground.

### Nps Spawner
Spawn AWSIM Npc using Rviz GUI tool.
Type and velocity of spawned Npc is specified from Rviz display.

<a href="./npc_spawner.png" data-lightbox="Npc Spawner" data-title="" data-alt="Npc Spawner"><img src="./npc_spawner.png"></a>

`awsim_rviz_plugins/NpsSpawner` tool gets position and orientation by dragging on the map displayed in Rviz, and publishes those as a `/awsim/awsim_rviz_plugins/npc_spawner/pose_with_covariance` topic.

AWSIM subscribes this topic and spawn Npc on the coordinates.

y-axis of destination position of teleport is calculated using ray-casting.  
The highest object at the specified x-z coordinates is considered the ground.

`awsim_rviz_plugins/NpsSpawnerStatus` display is entered Npc type (drop down list) and velocity, and publishes those as a `/awsim/awsim_rviz_plugins/npc_spawner/npc_name` and `/awsim/awsim_rviz_plugins/npc_spawner/npc_velocity` topic.

AWSIM subscribes those topics and specify type and velocity of spawned Npc.

Spawnable Npc is listed in `AutowreSimulationDemo/Function/AwsimRvizPluginsClient/`.  
AWSIM publishes name of spawnable Npc as `/awsim/awsim_rviz_plugins/npc_spawner/npc_name_list` topic.  
`awsim_rviz_plugins/NpsSpawnerStatus` display subscribes this topic and update drop down list of Npc type.  

### Overview

### Configuration
`Awsim Rviz Plugins` can be configured from `AwsimRvizPluginsClient` component.

<a href="./config.png" data-lightbox="Configulations" data-title="" data-alt="Configulations"><img src="./config.png"></a>

The configurable elements are listed in the following table:

2D Pose Teleport

| Parameter | Description |
|---|---|
| Ego Transform | Ego vehicle which is controlled by this plugin. |
| Ego Position Topic | Name of topic about position which Ego teleport. |

Npc Spawner Settings

| Parameter | Description |
|---|---|
| Npc Vehicle Prefabs | List of Npc vehicle.<br>Each element have identifier (`Name`) and reference (`Prefab`). |
| Npc Pedestrian Prefabs | List of Npc pedestrian.<br>Each element have identifier (`Name`) and reference (`Prefab`). |
| Npc Position Topic | Name of topic about position which Npc spawn. |
| Npc Name Topic | Name of topic about type of spawned Npc. |
| Npc Velocity Topic |Name of topic about velocity of spawned Npc. |
| Npc Name List Topic | Name of topic about list of vehicle names in `Rviz`. |

## Instruction
