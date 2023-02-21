# Trouble shooting

This document describes the most common errors encountered when working with AWSIm or autoware.

|Trouble|Solution|
|:--|:--|
|Massive output of Plugins errors|`git clone` the AWSIM repository again|
|error : `RuntimeError: error not set, at C:\ci\ws\src\ros2\rcl\rcl\src\rcl\node.c:262`|Set up environment variables and config around ROS2 correctly. For example:<br> - Environment variables<br> - `cyclonedds_config.xml`<br>
|`$ ros2 topic list` is not displayed|- your machine `ROS_DOMAIN_ID` is different<br>- `ROS2` is not sourced|
|Using AWSIM on Windows and Autoware on Ubuntu. <br> `$ ros2 topic list` is not displayed.|Allow the communication in Windows Firewall|
|self-driving stops in the middle of the road.|Check if your map data is correct (PointCloud, VectorMap, 3D fbx models)|
|Connecting AWSIM and Autoware results in bad network|Make ros local host-only. Include the following in the .bashrc (The password will be requested at terminal startup after OS startup.) <br><br> `export ROS_LOCALHOST_ONLY=1`<br>`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`<br><br>`if [ ! -e /tmp/cycloneDDS_configured ]; then`<br>`sudo sysctl -w net.core.rmem_max=2147483647`<br>`sudo ip link set lo multicast on`<br>`touch /tmp/cycloneDDS_configured`<br>`fi`|
|Lidar (colored pointcloud on RViz ) does not show. |Reduce processing load by following command. This can only be applied to Autoware's `awsim-stable` branch. <br><br> `cd <path_to_your_autoware_folder>`<br>`wget "https://drive.google.com/uc?export=download&id=11mkwfg-OaXIp3Z5c3R58Pob3butKwE1Z" -O patch.sh`<br>`bash patch.sh && rm patch.sh`|
|Error when starting AWSIM binary. `segmentation fault (core dumped)`|Check if yourNvidia drivers or Vulkan API are installed correctly|
|Initial pose does not match automatically. |Set initial pose manually. <br>![](Image_Initial_0.png)<br>![](Image_Initial_1.png)|
|Unity crashes and check the log for the cause of the error.|**Editor log file location**<br>Windows :<br> `C:\Users\username\AppData\Local\Unity\Editor\Editor.log`<br>Linux :<br> `~/.config/unity3d/.Editor.log` <br><br> **Player log file location**<br> Windows : `C:\Users\username\AppData\LocalLow\CompanyName\ProductName\output_log.txt`<br>Linux :<br>`~/.config/unity3d/CompanyName/ProductName/Player.log`<br><br>See also : [Unity Documentation - Log Files](https://docs.unity3d.com/2021.1/Documentation/Manual/LogFiles.html)|
|Safe mode dialog appears when starting UnityEditor. <br><br> or <br><br> error : `No usable version of libssl was found`|1. download libssl <br> `$ wget http://security.ubuntu.com/ubuntu/pool/main/o/openssl1.0/libssl1.0.0_1.0.2n-1ubuntu5.11_amd64.deb` <br><br> 2. install <br> `sudo dpkg -i libssl1.0.0_1.0.2n-1ubuntu5.11_amd64.deb`|