# Trouble shooting

|trouble|solution|
|:--|:--|
|Massive output of Plugins errors|git clone again|
|error : `RuntimeError: error not set, at C:\ci\ws\src\ros2\rcl\rcl\src\rcl\node.c:262`|Set up environment variables and config around ROS2 correctly.<br> - Environment variables<br> - `cyclonedds_config.xml`<br>- etc|
|`$ ros2 topic list` is not displayed|- ROS_DOMAIN_ID is different<br>- ROS not sourced|
|in AWSIM(windows) -> Autoware(ubuntu), <br> `$ ros2 topic list` is not displayed.|Allowed by windows firewall|
|self-driving stops in the middle of the road.|Maybe bad map data (PointCloud, VectorMap, fbx)6|
|Connecting AWSIM and Autoware results in bad network|Make ros local host-only. Include the following in the .bashrc (The password will be requested at terminal startup after OS startup.) <br><br> `export ROS_LOCALHOST_ONLY=1`<br>`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`<br><br>`if [ ! -e /tmp/cycloneDDS_configured ]; then`<br>`sudo sysctl -w net.core.rmem_max=2147483647`<br>`sudo ip link set lo multicast on`<br>`touch /tmp/cycloneDDS_configured`<br>`fi`|
