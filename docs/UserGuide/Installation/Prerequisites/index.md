### PC specs
<!-- copied from old -->
Please make sure that your machine meets the following requirements in order to run the simulation correctly:

|Required PC Specs||
|:--|:--|
|OS|Ubuntu 22.04|
|CPU|6 cores and 12 thread or higher|
|GPU|RTX2080Ti or higher|
|Nvidia Driver (Windows)|>=472.50|
|Nvidia Driver (Ubuntu 22)|>=515.43.04|

## Linux
### Localhost settings
<!-- TODO copied from old and added what I think is some additional help -->

The simulation is based on the appropriate network setting, which allows for trouble-free communication of the AWSIM simulation with the Autoware software.
To apply required localhost settings please add the following lines to `~/.bashrc` file.

``` bash
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ ! -e /tmp/cycloneDDS_configured ]; then
	sudo sysctl -w net.core.rmem_max=2147483647
	sudo ip link set lo multicast on
	touch /tmp/cycloneDDS_configured
fi
```

If you are having trouble with network connection consider replacing `export ROS_LOCALHOST_ONLY=1` with `export ROS_DOMAIN_ID=XX` where `XX` is a unique number in your network between 0 and 101 inclusive.
For more information about `ROS_DOMAIN_ID` head to official [documentation](https://docs.ros.org/en/rolling/Concepts/About-Domain-ID.html).

    - Install Git
    - Install Nvidia GPU driver (**gif** - how to test)
    - Install Vulkan Graphics Library
## Windows
<!-- TODO -->
### Localhost settings
<!-- TODO -->
### Install Git
<!-- TODO -->
### Install Nvidia GPU driver (**gif** - how to test)
<!-- TODO -->
### Install Vulkan Graphics Library
<!-- TODO -->
## Installing Autoware [70% current Launching Autoware]
<!-- TODO -->

### Launching Autoware
<!-- TODO copied from old -->

In order to configure and run the Autoware software with the AWSIM demo, please:

1. Download `map files (pcd, osm)` and unzip them.

    [Download Map files (pcd, osm)](https://github.com/tier4/AWSIM/releases/download/v1.1.0/nishishinjuku_autoware_map.zip){.md-button .md-button--primary}

2. Clone [Autoware](https://github.com/autowarefoundation/autoware) and move to the directory.
```
git clone https://github.com/autowarefoundation/autoware.git
cd autoware
```
3. Switch branch to `awsim-stable`. *NOTE: The latest `main` branch is for [ROS 2 humble](https://docs.ros.org/en/rolling/Releases/Release-Humble-Hawksbill.html).*
```
git checkout awsim-stable
```
4. Configure the environment. (Skip if Autoware environment has been configured before)
```
./setup-dev-env.sh
```
5. Create the `src` directory and clone external dependent repositories into it.
```
mkdir src
vcs import src < autoware.repos
```
6. Install dependent ROS packages.
```
source /opt/ros/humble/setup.bash
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```
7. Build the workspace.
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-w"
```
8. Launch Autoware.

    !!! warning

        `<your mapfile location>` must be changed arbitrarily. When specifying the path the `~` operator cannot be used - please specify absolute full path.
```
source install/setup.bash
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit map_path:=<your mapfile location>
```
![](Image_2.png)
