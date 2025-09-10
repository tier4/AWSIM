# Nps Spawner
Spawn AWSIM Npc using Rviz GUI tool.
Type and velocity of spawned Npc is specified from Rviz display.

<a href="./npc_spawner.png" data-lightbox="Npc Spawner" data-title="Npc Spawner" data-alt="Npc Spawner"><img src="./npc_spawner.png"></a>

`awsim_rviz_plugins/NpsSpawner` tool gets position and orientation by dragging on the map displayed in Rviz, and publishes those as a `/awsim/awsim_rviz_plugins/npc_spawner/pose_with_covariance` topic.

AWSIM subscribes this topic and spawn Npc on the coordinates.

y-axis of destination position of teleport is calculated using ray-casting.  
The highest object at the specified x-z coordinates is considered the ground.

`awsim_rviz_plugins/NpsSpawnerStatus` display is entered Npc type (drop down list) and velocity, and publishes those as a `/awsim/awsim_rviz_plugins/npc_spawner/npc_name` and `/awsim/awsim_rviz_plugins/npc_spawner/npc_velocity` topic.

AWSIM subscribes those topics and specify type and velocity of spawned Npc.

Spawnable Npc is listed in `AutowreSimulationDemo/Function/AwsimRvizPluginsClient/`.  
AWSIM publishes name of spawnable Npc as `/awsim/awsim_rviz_plugins/npc_spawner/npc_name_list` topic.  
`awsim_rviz_plugins/NpsSpawnerStatus` display subscribes this topic and update drop down list of Npc type.  

## How to use
1. On Rviz, Click the plus button on the toolbar and select `awsim_rviz_plugins/NpcSpawner` from the list.

<a href="./tool_bar.png" data-lightbox="Tool Bar" data-title="Tool Bar" data-alt="Tool Bar"><img src="./tool_bar.png"></a>

2. On Rviz, Click on `Npc Spawner` button from the toolbar and select it.
3. On AWSIM, Play binary or Play scene.
4. On the map displayed in Rviz, drag the cursor to the location and orientation where you want to move the EGO.

If you want to change type and velocity of spawned Npc, do the following:

1. On Rviz, Click the `Add` button on the `Display` panel and select `awsim_rviz_plugins/NpcSpawnerStatus` from the list.

<a href="./status_panel.png" data-lightbox="Status Panel" data-title="Status Panel" data-alt="Status Panel"><img src="./status_panel.png"></a>

2. On Rviz, Change the value of `Npc Type` and `Velocity [km/h]`.