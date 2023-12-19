

# Using OpenSCENARIO
  
!!! warning
    
    Running AWSIM with scenario_simulator_v2 is still a prototype, so stable running is not guaranteed.

Below you can find instructions on how to setup the OpenSCENARIO execution using [scenario_simulator_v2](https://github.com/tier4/scenario_simulator_v2) with AWSIM as a simulator
The instruction assumes using the Ubuntu OS.

## Prerequisites
Follow [Setup Unity Project tutorial](https://tier4.github.io/AWSIM/GettingStarted/SetupUnityProject/)

## Build Autoware with `scenario_simulator_v2`

In order to configure the Autoware software with the AWSIM demo, please:

1. Clone RobotecAI's [Autoware](https://github.com/RobotecAI/autoware-1/tree/awsim-ss2-stable) and move to the directory.
   ```
   git clone git@github.com:RobotecAI/autoware-1.git
   cd autoware
   ```
2. Check out to the `awsim-ss2-stable` branch
   ```
   git checkout awsim-ss2-stable
   ```
3. Configure the environment. (Skip if Autoware environment has been configured before)
   ```
   ./setup-dev-env.sh
   ```
4. Create the `src` directory and clone external dependent repositories into it.
   ```
   mkdir src
   vcs import src < autoware.repos
   vcs import src < simulator.repos
   ```
5. Download `shinjuku_map.zip`  
   [archive](https://github.com/tier4/AWSIM/releases/download/v1.2.0/shinjuku_map.zip){.md-button .md-button--primary} 
 
6. Unzip it to `src/simulator` directory
   ```
   unzip <Download directory>/shinjuku_map.zip -d src/simulator
   ```
7. Install dependent ROS packages.
   ```
   source /opt/ros/humble/setup.bash
   rosdep update
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```
8. Build the workspace.
   ```
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-w"
   ```

## Running the demo

1. Download `AWSIM_v1.2.0_ss2.zip` & Run  
   [archive](https://github.com/tier4/AWSIM/releases/download/v1.2.0/AWSIM_v1.2.0_ss2.zip){.md-button .md-button--primary} 

2. Launch `scenario_test_runner`.
   ```
   source install/setup.bash
   ros2 launch scenario_test_runner scenario_test_runner.launch.py                        \
   architecture_type:=awf/universe  record:=false                                         \
   scenario:='$(find-pkg-share scenario_test_runner)/scenario/sample_awsim.yaml'          \
   sensor_model:=awsim_sensor_kit  vehicle_model:=sample_vehicle                          \
   launch_simple_sensor_simulator:=false autoware_launch_file:="e2e_simulator.launch.xml" \
   initialize_duration:=260 port:=8080
   ```
   ![ss2_awsim.png](ss2_awsim.png)

## Troubleshooting

In case of problems, make sure that the [regular demo](https://tier4.github.io/AWSIM/GettingStarted/QuickStartDemo/) work well with the Autoware built above. Follow the [troubleshooting page](https://tier4.github.io/AWSIM/DeveloperGuide/TroubleShooting/) there if necessary.

## Appendix
- [AWSIM ROS2 topic list](../../Components/ROS2/ROS2TopicList/index.md)
