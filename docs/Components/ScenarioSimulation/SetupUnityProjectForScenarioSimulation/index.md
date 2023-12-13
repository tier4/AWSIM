# Running AWSIM from Unity Editor with `scenario_simulator_v2`

Below you can find instructions on how to setup the scenario execution using `scenario_simulator_v2` with AWSIM run from Unity Editor as a simulator
The instruction assumes using the Ubuntu OS.

## Prerequisites
1. Build Autoware by following ["Build Autoware with `scenario_simulator_v2`" section from the scenario simulator and AWSIM quick start guide](https://tier4.github.io/AWSIM/GettingStarted/UsingOpenSCENARIO)

2. Follow [Setup Unity Project tutorial](https://tier4.github.io/AWSIM/GettingStarted/SetupUnityProject/)

## Running the demo

1. Open AutowareSimulationScenarioSimulator.unity scene placed under `Assets/AWSIM/Scenes/Main` directory
2. Run the simulation by clicking `Play` button placed at the top section of Editor.
3. Launch `scenario_test_runner`.
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

## Other sample scenarios

### Conventional traffic lights demo

This scenario controls traffic signals in the scene based on OpenSCENARIO. It can be used to verify whether traffic light recognition pipeline works well in Autoware.

```
ros2 launch scenario_test_runner scenario_test_runner.launch.py                                           \
architecture_type:=awf/universe  record:=false                                                            \
scenario:='$(find-pkg-share scenario_test_runner)/scenario/sample_awsim_conventional_traffic_lights.yaml' \
sensor_model:=awsim_sensor_kit  vehicle_model:=sample_vehicle                                             \
launch_simple_sensor_simulator:=false autoware_launch_file:="e2e_simulator.launch.xml"                    \
initialize_duration:=260 port:=8080
```

### V2I traffic lights demo

This scenario publishes V2I traffic signals information based on OpenSCENARIO. It can be used to verify Autoware responds to V2I traffic lights information correctly.

```
ros2 launch scenario_test_runner scenario_test_runner.launch.py                                  \
architecture_type:=awf/universe  record:=false                                                   \
scenario:='$(find-pkg-share scenario_test_runner)/scenario/sample_awsim_v2i_traffic_lights.yaml' \
sensor_model:=awsim_sensor_kit  vehicle_model:=sample_vehicle                                    \
launch_simple_sensor_simulator:=false autoware_launch_file:="e2e_simulator.launch.xml"           \
initialize_duration:=260 port:=8080
```


