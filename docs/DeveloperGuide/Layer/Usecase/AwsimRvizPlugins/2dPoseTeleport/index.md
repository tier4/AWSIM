# 2D Pose Teleport
Teleport AWSIM EGO using Rviz GUI tool.
![2D Pose Teleport](./2d_pose_teleport.png)

`awsim_rviz_plugins/2dPoseTeleport` tool gets position and orientation by dragging on the map displayed in Rviz, and publishes those as a `/awsim/awsim_rviz_plugins/pose_teleport/pose_with_covariance` topic.

AWSIM subscribes this topic and updates the coordinates of the EGO.

y-axis of destination position of teleport is calculated using ray-casting.  
The highest object at the specified x-z coordinates is considered the ground.

## How to use
1. On Rviz, Click the plus button on the toolbar and select `awsim_rviz_plugins/2dPoseTeleport` from the list.

![2D Pose Teleport Setup](./tool_bar.png)

2. On Rviz, Click on `2D Pose Teleport` button from the toolbar and select it.
3. On AWSIM, Play binary or Play scene.
4. On the map displayed in Rviz, drag the cursor to the location and orientation where you want to move the EGO.

If `Autoware (pilot-auto)` is running with, press the `Initialize with GNSS` button to perform localilization again.