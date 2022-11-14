# Trouble shooting

|trouble|solution|
|:--|:--|
|Massive output of Plugins errors|git clone again|
|error : `RuntimeError: error not set, at C:\ci\ws\src\ros2\rcl\rcl\src\rcl\node.c:262`|Set up environment variables and config around ROS2 correctly.<br> - Environment variables<br> - `cyclonedds_config.xml`<br>- etc|
|`$ ros2 topic list` is not displayed|- ROS_DOMAIN_ID is different<br>- ROS not sourced|
|in AWSIM(windows) -> Autoware(ubuntu), <br> `$ ros2 topic list` is not displayed.|Allowed by windows firewall|
|self-driving stops in the middle of the road.|Maybe bad map data (PointCloud, VectorMap, fbx)|
|Connecting AWSIM and Autoware results in bad network|Make ros local host-only. Include the following in the .bashrc (The password will be requested at terminal startup after OS startup.) <br><br> `export ROS_LOCALHOST_ONLY=1`<br>`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`<br><br>`if [ ! -e /tmp/cycloneDDS_configured ]; then`<br>`sudo sysctl -w net.core.rmem_max=2147483647`<br>`sudo ip link set lo multicast on`<br>`touch /tmp/cycloneDDS_configured`<br>`fi`|
|Lidar (colored pointcloud on RViz ) does not show. |Reduce processing load by following command. This can only be applied to `awsim-stable` branch. <br><br> `cd <path_to_your_autoware_folder>`<br>`wget "https://drive.google.com/uc?export=download&id=11mkwfg-OaXIp3Z5c3R58Pob3butKwE1Z" -O patch.sh`<br>`bash patch.sh && rm patch.sh`|
|Error when starting AWSIM binary. `segmentation fault (core dumped)`|Correctly install Nvidia drivers or vulkan API.|
|Initial pose does not match automatically. |Set initial pose manually. <br>![](Image_Initial_0.png)<br>![](Image_Initial_1.png)|
|Unity crashes and check the log for the cause of the error.|**Editor**<br>Windows :<br> `C:\Users\username\AppData\Local\Unity\Editor\Editor.log`<br>Linux :<br> `~/.config/unity3d/.Editor.log` <br><br> **Player**<br> Windows : `C:\Users\username\AppData\LocalLow\CompanyName\ProductName\output_log.txt`<br>Linux :<br>`~/.config/unity3d/CompanyName/ProductName/Player.log`<br><br>See also : [Unity Documentation - Log Files](https://docs.unity3d.com/2021.1/Documentation/Manual/LogFiles.html)|

