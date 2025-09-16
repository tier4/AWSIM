## ROS2 for unity
[*Ros2ForUnity*](https://github.com/RobotecAI/ros2-for-unity) (`R2FU`) module is a communication solution that effectively connects *Unity* and the *ROS2* ecosystem, maintaining a strong integration.
Unlike other solutions, it doesn't rely on bridging communication but rather utilizes the *ROS2* middleware stack (specifically the `rcl` layer and below), enabling the inclusion of *ROS2* nodes within *Unity* simulations.

`R2FU` is used in *AWSIM* for many reasons.
First of all, because it offers high-performance integration between *Unity* and *ROS2*, with improved throughput and lower latencies compared to bridging solutions.
It provides real *ROS2* functionality for simulation entities in *Unity*, supports standard and custom messages, and includes convenient abstractions and tools, all wrapped as a *Unity* asset.
For a detailed description, please see [*README*](https://github.com/RobotecAI/ros2-for-unity/blob/master/README.md).

!!! warning

    To avoid internal conflicts between the standalone libraries, and sourced ones, *ROS2* instance shouldn't be sourced before running AWSIM or the *Unity* Editor.

!!! question "Can't see topics"
    There are no errors but I can't see topics published by `R2FU`

    - Make sure your DDS ([Localhost settings](../../../GettingStarted/QuickStartDemo/#localhost-settings)) config is correct.
    - Sometimes *ROS2* daemon brakes up when changing network interfaces or *ROS2* version.
Try to stop it forcefully (`pkill -9 ros2_daemon`) and restart (`ros2 daemon start`).

## Awsim ROS2 node

[AwsimRos2Node](https://github.com/tier4/AWSIM-v2.0.0-draft/blob/main/Assets/Awsim/Scripts/Common/Ros2/AwsimRos2Node.cs) static class is a ROS2 node that can be commonly used in AWSIM. The design of AwsimRos2Node provides the following advantages.

- Time sources used in ROS2 can be centrally managed, modified, and reflected.
- A single node can be used to aggregate topics to be Pub/Subbed in AWSIM.
- Ability to add and delete pub/subs on topics and services and retrieve times.

ROS2 node name and time source can be specified at initialization.
```
AwsimRos2Node.Initialize("AWSIM", TimeSourceType.Ros2);
```

## Clock

[ClockRos2Publisher](https://github.com/tier4/AWSIM-v2.0.0-draft/blob/main/Assets/Awsim/Scripts/Common/Ros2/ClockRos2Publisher.cs) allows the publication of the simulation time from the clock operating within AWSIM. The current time is retrived from a `TimeSource` object via the `AwsimRos2Node`. The AWSIM provides convenient method for selecting the appropriate time source type as well as the flexibility to implement custom `TimeSources` tailored to specific user requirements.

## Timesource

List of Time Sources

| Type | TimeSouceType enum value | Driven by | Start Value | Affected by Time Scale | Remarks |
|:-|:-|:-|:-|:-|:-|
| Unity | 0 | UnityEngine.Time | 0 | yes | |
| External | 1 | externally | depends on external source | no | used by the [scenario simulator v2](../../ScenarioSimulation/PreparingTheConnectionBetweenAWSIMAndScenarioSimulator/) |
| DotnetSystem | 2 | System.DateTime | UNIX epoch| yes| starts with UNIX epoch time and progresses with System.DateTime scaled by AWSIM time scale |
| DotnetSimulation | 3 | System.DateTime | 0 | yes | starts with zero value and progresses with System.DateTime scaled by AWSIM time scale |
| Ros2 | 4 | ROS2.Clock | UNIX epoch (by default)| no | uses ROS 2 time |

<br>

## Default message types
The basic *ROS2* msgs types that are supported in *AWSIM* by default include:

- [common_interfaces](https://index.ros.org/r/common_interfaces/github-ros2-common_interfaces/):
    - [`std_msgs`](https://index.ros.org/p/std_msgs/github-ros2-common_interfaces/#humble).
    - [`geometry_msgs`](https://index.ros.org/p/geometry_msgs/github-ros2-common_interfaces/#humble),
    - [`sensor_msgs`](https://index.ros.org/p/sensor_msgs/github-ros2-common_interfaces/#humble),
    - [`nav_msgs`](https://index.ros.org/p/nav_msgs/github-ros2-common_interfaces/#humble),
    - [`diagnostic_msgs`](https://index.ros.org/p/diagnostic_msgs/github-ros2-common_interfaces/#humble),
- [rcl_interfaces](https://index.ros.org/r/rcl_interfaces/github-ros2-rcl_interfaces/):
    - [`builtin_interfaces`](https://index.ros.org/p/builtin_interfaces/github-ros2-rcl_interfaces/#humble),
    - [`action_msgs`](https://index.ros.org/p/action_msgs/github-ros2-rcl_interfaces/#humble),
    - [`rosgraph_msgs`](https://index.ros.org/p/rosgraph_msgs/github-ros2-rcl_interfaces/#humble),
    - [`test_msgs`](https://index.ros.org/p/test_msgs/github-ros2-rcl_interfaces/#humble).
- [autoware_msgs](https://github.com/autowarefoundation/autoware):
    - [`autoware_common_msgs`](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_common_msgs),
    - [`autoware_control_msgs`](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_control_msgs),
    - [`autoware_localization_msgs`](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_localization_msgs),
    - [`autoware_map_msgs`](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_map_msgs),
    - [`autoware_perception_msgs`](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_perception_msgs),
    - [`autoware_planning_msgs`](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_planning_msgs),
    - [`autoware_sensing_msgs`](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_sensing_msgs),
    - [`autoware_system_msgs`](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_system_msgs),
    - [`autoware_vehicle_msgs`](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_vehicle_msgs).
- [tier4_autoware_msgs](https://github.com/tier4/tier4_autoware_msgs):
    - [`tier4_control_msgs`](https://github.com/tier4/tier4_autoware_msgs/tree/tier4/universe/tier4_control_msgs),
    - [`tier4_vehicle_msgs`](https://github.com/tier4/tier4_autoware_msgs/tree/tier4/universe/tier4_vehicle_msgs).
- Others:
    - [`tf2_msgs`](https://index.ros.org/p/tf2_msgs/github-ros2-geometry2/#humble),
    - [`unique_identifier_msgs`](https://index.ros.org/p/unique_identifier_msgs/github-ros2-unique_identifier_msgs/#humble).

In order for the message package to be used in *Unity*, its `*.dll` and `*.so` libraries must be generated using `R2FU`.

!!! tip "Custom message"
    If you want to generate a custom message to allow it to be used in *AWSIM* please read [this tutorial](../AddCustomRos2Message/index.md).
