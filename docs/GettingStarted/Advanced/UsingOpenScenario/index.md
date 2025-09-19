!!!info
    If you want to setup `Unity` scene with `Scenario_simulator_v2`, see [here](../../../DeveloperGuide/Layer/Usecase/ScenarioSimulatorConnection/index.md).

## Overview

``` mermaid 
flowchart LR
    AWSIM <--ROS2--> Autoware
    AWSIM <--ZeroMQ--> ScenarioSimulator
    Autoware <--ROS2--> ScenarioSimulator
```


## OpenSCENARIO demo simulation tutorial

### 1. Preparation

1. Operating System  
    - Ubuntu 22.04 is required.  
        - Due to the reliance on ROS 2 Humble, which is officially supported only on Ubuntu 22.04 (Jammy).  
        - Using other distributions (such as 20.04 or 24.04) may result in unexpected build or runtime issues due to lack of binary support and untested dependencies.

1. Hardware Requirements  
    - Memory
        - At least 32 GB of RAM is recommended.
        - If your system has less than 32 GB, consider the following options to avoid out-of-memory (OOM) errors during the build:
            - Enable swap space (at least 16 GB recommended).
            - Reduce the number of parallel jobs by adding `MAKEFLAGS="-j1"` or `--parallel-workers 1` when building with colcon.
    - GPU
        - It is recommended to use an NVIDIA GPU with 8 GB of VRAM or more.  
        - This is important for running AWSIM smoothly and ensuring proper performance of Autoware's perception modules.

1. Basic Package Setup  
    - Before proceeding, make sure the package index is up-to-date and that essential tools like git are installed:  

    ```
    sudo apt -y update
    sudo apt -y install git 
    ```
    - git is required for cloning repositories such as Autoware, AWSIM.

1. Unity Installation  
    AWSIM runs on the Unity engine, so you need to install **Unity Hub** and the required Unity Editor version.
    - Install Unity Hub  
        Follow the official guide to install Unity Hub on your platform:  
        https://docs.unity3d.com/hub/manual/InstallHub.html#install-hub-linux

    - Install Unity Editor  
       Use Unity Hub to install the following version of Unity:
       - **Unity 6000.0.34f1**

1. To improve the performance and stability of DDS-based communication, it's recommended to apply the following optimizations.

    - Set the system optimizations by adding this code to the very bottom of your ~/.bashrc file:
    ```
    if [ ! -e /tmp/cycloneDDS_configured ]; then
        sudo sysctl -w net.core.rmem_max=2147483647
        sudo ip link set lo multicast on
        touch /tmp/cycloneDDS_configured
    fi
    ```
    - After editing, apply the changes:
    ```
    source ~/.bashrc
    ```

### 2. Install Autoware with Scenario simulator v2

1. Clone [Autoware](https://github.com/autowarefoundation/autoware) and move to the directory.
    ```
    git clone https://github.com/autowarefoundation/autoware.git &&
    cd autoware
    ```

1. Configure the environment.
    ```
    ./setup-dev-env.sh

    ```
    When you run this script, it will prompt several questions. The recommended answers are:
    
    - `Are you sure you want to run setup? [y/N]` → **y**
        - This starts the environment setup process.
    
    - `Install NVIDIA libraries? [y/N]` → **y**  
        - This installs GPU driver, CUDA, cuDNN, TensorRT.  
        - You may choose **n** if you prefer to install them manually or already have them installed.
    
    - `Download artifacts? [y/N]` → **y**
        - Downloads prebuilt artifacts.

1. Confirm the version in `simulator.repos`.  
Before importing repositories, check if the version specified in `simulator.repos` is the latest.  
If it's outdated, update it to the latest stable release to avoid compatibility issues.  
At the time of writing, the latest version is `16.6.1`.  
`simulator.repos`:
    ```
    repositories:
        simulator/scenario_simulator:
            type: git
            url: https://github.com/tier4/scenario_simulator_v2.git
            version: 16.6.1
    ```

1. Create the `src` directory and clone external dependent repositories into it.
    ```
    mkdir src &&
    vcs import src < autoware.repos &&
    vcs import src < simulator.repos
    ```

1. Download [shinjuku_map.zip](https://github.com/tier4/AWSIM/releases/download/v1.3.0/shinjuku_map.zip)

1. Unzip it to `src/simulator` directory.
    ```
    unzip <Download directory>/shinjuku_map.zip -d src/simulator.
    ```

1. Install dependent ROS packages.
    ```
    source /opt/ros/humble/setup.bash &&
    rosdep update &&
    rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
    ```

1. Build the workspace.
    ```
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-w"
    ```

### 3. Running the demo.

1. Use Unity Hub to open the AWSIM2.0 project.

1. Open the `IntegrateScenarioSimulatorDemo` scene.

    - In the **Project** window, open the following scene:  
        - `Assets/Scenes/ScenarioSimulatorDemo/IntegrateScenarioSimulatorDemo.unity`

    - This scene contains the ScenarioSimulatorClient setup for connecting to ScenarioSimulatorV2.

1. Run AWSIM2.0.

    - Click the **Play** button at the top of the Unity Editor to start the simulation.  

1. Launch `scenario_test_runner`.

    ```
    source install/setup.bash
    ros2 launch scenario_test_runner scenario_test_runner.launch.py                        \
    architecture_type:=awf/universe/20250130  record:=false                                         \
    scenario:='$(find-pkg-share scenario_test_runner)/scenario/sample_awsim.yaml'          \
    sensor_model:=awsim_sensor_kit  vehicle_model:=sample_vehicle                          \
    launch_simple_sensor_simulator:=false autoware_launch_file:="e2e_simulator.launch.xml" \
    initialize_duration:=260 port:=8080
    ```