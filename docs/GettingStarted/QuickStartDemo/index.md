# Quick Start Demo

This page describes a Self-driving simulation connected to the AWSIM demo binary and Autoware. Use one Ubuntu PC.

![](Image_top.png)

The simulation provided in the AWSIM demo has the following settings.

|AWSIM Demo Settings||
|:--|:--|
|Vehicle|Lexus RX 450h|
|Environment|Japan Tokyo Nishishinjuku|
|Sensors|Gnss * 1<br> IMU * 1<br> LiDAR * 1<br> Traffic camera * 1|
|Traffic|Randomized traffic|


## 1. Check the required specifications
PC specifications that meet the following requirements are required.

|Required PC Specs||
|:--|:--|
|OS|Ubutnu 20.04|
|CPU|6cores and 12thread or higher|
|GPU|RTX2080Ti or higher|
|Nvidia Driver (Windows)|>=472.50|
|Nvidia Driver (Linux)|>=460.27.03|


## 2. ROS localhost settings
    
Add the following to `.bashrc`
```
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ ! -e /tmp/cycloneDDS_configured ]; then
	sudo sysctl -w net.core.rmem_max=2147483647
	sudo ip link set lo multicast on
	touch /tmp/cycloneDDS_configured
fi
```

## 3. Launch AWSIM Demo binary

1. Install Nvidia GPU driver (Skip if already installed).
    1. Add Nvidia driver to apt repository
    ```
    sudo add-apt-repository ppa:graphics-drivers/ppa
    ```
    2. Check supported Nvidia driver versions.
    ```
    sudo apt update
    ```
    3. Install the recommended version.
    ```
    sudo ubuntu-drivers autoinstall
    ```
    4. reboot and check nvidia-smi.
    ```
    $ nvidia-smi 
    Fri Oct 14 01:41:05 2022       
    +-----------------------------------------------------------------------------+
    | NVIDIA-SMI 515.65.01    Driver Version: 515.65.01    CUDA Version: 11.7     |
    |-------------------------------+----------------------+----------------------+
    | GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
    | Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
    |                               |                      |               MIG M. |
    |===============================+======================+======================|
    |   0  NVIDIA GeForce ...  Off  | 00000000:01:00.0  On |                  N/A |
    | 37%   31C    P8    30W / 250W |    188MiB / 11264MiB |      3%      Default |
    |                               |                      |                  N/A |
    +-------------------------------+----------------------+----------------------+

    +-----------------------------------------------------------------------------+
    | Processes:                                                                  |
    |  GPU   GI   CI        PID   Type   Process name                  GPU Memory |
    |        ID   ID                                                   Usage      |
    |=============================================================================|
    |    0   N/A  N/A      1151      G   /usr/lib/xorg/Xorg                133MiB |
    |    0   N/A  N/A      1470      G   /usr/bin/gnome-shell               45MiB |
    +-----------------------------------------------------------------------------+
    ```

2. Install Vulkun Graphics Library (Skip if already installed).
    1. Update environment.
    ```
    sudo apt update
    ```
    2. Install libvulkan1.
    ```
    sudo apt install libvulkan1
    ```

3. Download and Run AWSIM Demo binary.
    1. Download `AWSIM_vXXX.zip` and Unzip.

        [Download AWSIM Demo for ubuntu](https://github.com/tier4/AWSIM/releases/download/v1.0.1/AWSIM_v1.0.1.zip){.md-button .md-button--primary}
    
    2. Allow permission.

        Check the `Execute` checkbox.

        ![](Image_1.png)

    3. Launch `AWSIM.x86_64`.
        ```
        ./<path to AWSIM folder>/AWSIM.x86_64
        ``` 
        
        !!! warning
        
            It may take a little time to start up.
        ![](Image_0.png)

## 4. Launch Autoware

1. Download `map files(pcd, osm)` and unzip.

    [Download Map files (pcd, osm)](https://github.com/tier4/AWSIM/releases/download/v1.0.0/nishishinjuku_autoware_map.zip){.md-button .md-button--primary}

2. Clone [autoware](https://github.com/autowarefoundation/autoware) and move to the directory.
```
git clone https://github.com/autowarefoundation/autoware.git
cd autoware
```
3. Switch branches to `awsim-stable`. (The latest `main` branch may work, but `awsim-stable` will definitely work.)
```
git checkout awsim-stable
```
4. Setup environment. If you have already prepared an Autoware environment, you don't need to run this.
```
./setup-dev-env.sh
```
5. Create the `src` directory and clone repositories into it.
```
mkdir src
vcs import src < autoware.repos
```
6. Install dependent ROS packages.
```
source /opt/ros/galactic/setup.bash
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```
7. Build the workspace.
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-w"
```
8. Launch Autoware.
    
    `<your mapfile location>` should be changed arbitrarily. You cannot use `~` to specify the path.
```
source install/setup.bash
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit map_path:=<your mapfile location>
```
![](Image_2.png)

## 5. Let's Self-Driving simulation
1. Launch both AWSIM and Autoware.
![](Image_top.png)
2. Automatically set 2D Pose Estimate with RViz.
![](Image_Initial.png)
3. Manually set 2D Goal Pose with RViz.
![](Image_goal_0.png)
![](Image_goal_1.png)
4. Set 2D Checkpoint Pose (Optional).
![](Image_checkpoint_0.png)
Path generated.
![](Image_path.png)
5. Engage self-driving.

Open new terminal and engage self-driving !!

```
ros2 topic pub /autoware/engage autoware_auto_vehicle_msgs/msg/Engage '{engage: True}' -1
```
![](Image_running.png)

## Appendix
- [AWSIM ROS2 topic list](../../Components/ROS2/ROS2TopicList/index.md)
