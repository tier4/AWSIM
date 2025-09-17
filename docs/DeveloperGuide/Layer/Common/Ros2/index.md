# Ros2

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

    - Make sure your DDS ([Localhost settings](../../../../GettingStarted/QuickStartDemo/#localhost-settings)) config is correct.
    - Sometimes *ROS2* daemon brakes up when changing network interfaces or *ROS2* version.
Try to stop it forcefully (`pkill -9 ros2_daemon`) and restart (`ros2 daemon start`).

## Awsim ROS2 node

[AwsimRos2Node](https://github.com/tier4/AWSIM/blob/docs/ROS2_modify/Assets/Awsim/Scripts/Common/Ros2/AwsimRos2Node.cs) static class is a ROS2 node that can be commonly used in AWSIM. The design of AwsimRos2Node provides the following advantages.

- Time sources used in ROS2 can be centrally managed, modified, and reflected.
- A single node can be used to aggregate topics to be Pub/Subbed in AWSIM.
- Ability to add and delete pub/subs on topics and services and retrieve times.

ROS2 node name and time source can be specified at initialization.
```
AwsimRos2Node.Initialize("AWSIM", TimeSourceType.Ros2);
```

## Clock

[ClockRos2Publisher](https://github.com/tier4/AWSIM/blob/main/Assets/Awsim/Scripts/Common/Ros2/ClockRos2Publisher.cs) allows the publication of the simulation time from the clock operating within AWSIM. The current time is retrived from a `TimeSource` object via the `AwsimRos2Node`. The AWSIM provides convenient method for selecting the appropriate time source type as well as the flexibility to implement custom `TimeSources` tailored to specific user requirements.

## Timesource

List of Time Sources

| Type | TimeSouceType enum value | Driven by | Start Value | Affected by Time Scale | Remarks |
|:-|:-|:-|:-|:-|:-|
| Unity | 0 | UnityEngine.Time | 0 | yes | |
| External | 1 | externally | depends on external source | no | used by the [scenario simulator v2](../../Usecase/ScenarioSimulatorConnection/) |
| DotnetSystem | 2 | System.DateTime | UNIX epoch| yes| starts with UNIX epoch time and progresses with System.DateTime scaled by AWSIM time scale |
| DotnetSimulation | 3 | System.DateTime | 0 | yes | starts with zero value and progresses with System.DateTime scaled by AWSIM time scale |
| Ros2 | 4 | ROS2.Clock | UNIX epoch (by default)| no | uses ROS 2 time |

<br>

## Default message types
The basic *ROS2* msgs types that are supported in *AWSIM* by default include:

- [common_interfaces](https://index.ros.org/r/common_interfaces/):
    - [`std_msgs`](https://index.ros.org/p/std_msgs/).
    - [`geometry_msgs`](https://index.ros.org/p/geometry_msgs/),
    - [`sensor_msgs`](https://index.ros.org/p/sensor_msgs/),
    - [`nav_msgs`](https://index.ros.org/p/nav_msgs/),
    - [`diagnostic_msgs`](https://index.ros.org/p/diagnostic_msgs/),
- [rcl_interfaces](https://index.ros.org/r/rcl_interfaces/):
    - [`builtin_interfaces`](https://index.ros.org/p/builtin_interfaces/),
    - [`action_msgs`](https://index.ros.org/p/action_msgs/),
    - [`rosgraph_msgs`](https://index.ros.org/p/rosgraph_msgs/),
    - [`test_msgs`](https://index.ros.org/p/test_msgs/).
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
    - [`tf2_msgs`](https://index.ros.org/p/tf2_msgs/),
    - [`unique_identifier_msgs`](https://index.ros.org/p/unique_identifier_msgs/).

In order for the message package to be used in *Unity*, its `*.dll` and `*.so` libraries must be generated using `R2FU`.

## AddCustomRos2Message
If you want to use custom message in *AWSIM*, you need to generate the appropriate files, to do this you have to build `ROS2ForUnity` yourself - please follow the steps below. Remember to start with [prerequisities](#prerequisites) though.

!!! warning "custom_msgs"
    In order to simplify this tutorial, the name of the package containing the custom message is assumed to be `custom_msgs` - remember to replace it with the name of your package.

### Prerequisites

`ROS2ForUnity` depends on a [ros2cs](https://github.com/RobotecAI/ros2cs) - a *C# .NET* library for *ROS2*.<br>
This library is already included so you don't need to install it, but there are a few prerequisites that must be resolved first.

Please select your system and resolve all prerequisites:

=== "Ubuntu"
    - [`ros2cs` prerequisites for *Ubuntu*](https://github.com/RobotecAI/ros2cs/blob/master/README-UBUNTU.md#prerequisites)
    - *ROS2* version is `humble` and is located in `/opt/ros/humble`
    - Your package with custom message is located in the home directory `~/custom_msgs` or is hosted on *git* repository.
    - *Shell* - commands have to be executed from the `bash` shell 
<!-- === "Windows"
    - [`ros2cs` prerequisites for *Windows*](https://github.com/RobotecAI/ros2cs/blob/1.2.0/README-WINDOWS.md#prerequisites)

        !!! question
            Tests are not working (`'charmap'` codec can't decode byte) on *Windows* - look at troubleshooting [here](https://github.com/RobotecAI/ros2cs/blob/master/README-UBUNTU.md#prerequisites) -->
            
    - *ROS2* version is `humble` and is located in  `C:\ros2_humble`
    - Your package with custom message package is located in the home directory `C:\custom_msgs` or is hosted on *git* repository.
    - *Shell* - commands should be executed from the `powershell` shell

### 1. Workspace preparation

1. Clone `ROS2ForUnity` repository by execute command:

    === "Ubuntu"
        ```
        git clone https://github.com/RobotecAI/ros2-for-unity ~/
        ```
        !!! warning 
            The cloned `ROS 2 For Unity` repository must be located in the home directory `~/`.
    <!-- === "Windows"
        ```
        git clone https://github.com/RobotecAI/ros2-for-unity /C
        ```
        !!! warning 
            The cloned `ROS 2 For Unity` repository must be located in the home directory `C:\`. -->

2. Pull dependent repositories by execute commands:

    === "Ubuntu"
        ```bash
        cd ~/ros2-for-unity &&
        ./opt/ros/humble/setup.bash &&
        ./pull_repositories.sh
        ```
   

    <!-- === "Windows"
        ```powershell
        cd C:\ros2-for-unity
        C:\ros2_humble\local_setup.ps1
        .\pull_repositories.ps1
        ``` -->
        

### 2. Setup `custom_msgs` package
The method to add a custom package to build depends on where it is located. The package can be on your local machine or just be hosted on a *git* repository.<br>
Please, choose the appropriate option and follow the instructions.

  <!-- 1. Package hosted on *git* repository - listing them in `ros2_for_unity_custom_messages.repos` file,
  2. Package contained on local machine - manually inserting them in `src/ros2cs` directory. -->

#### 2.1. Package contained on local machine

1. Copy the `custom_msgs` package with custom message to the folder to `src/ros2cs/custom_messages` directory
    
    === "Ubuntu"
        ```bash
        cp -r ~/custom_msgs ~/ros2-for-unity/src/ros2cs/custom_messages/
        ```

    <!-- === "Windows"
        ```powershell
        Copy-Item 'C:\custom_msgs' -Destination 'C:\ros2-for-unity\src\custom_messages'
        ``` -->

#### 2.2. Package hosted on git repository

1. Open `ros2-for-unity/ros2_for_unity_custom_messages.repos` file in editor.
2. Modify the contents of the file shown below, **uncomment** and **set**:

    - `<package_name>` - to your package name - so in this case `custom_msgs`,
    - `<repo_url>` - to repository address,  
    - `<repo_branch>` - to desired branch.
    ```
    repositories:
    #  src/ros2cs/custom_messages/<package_name>:
    #    type: git
    #    url: <repo_url>
    #    version: <repo_branch>
    ```


    !!! example
        Below is an example of a file configured to pull 2 packages (`custom_msgs`,`autoware_msgs`) of messages hosted on a *git* repository.
        ```
        # NOTE: Use this file if you want to build with custom messages that reside in a separate remote repo.
        # NOTE: use the following format

        repositories:
            src/ros2cs/custom_messages/custom_msgs:
                type: git
                url: https://github.com/tier4/custom_msgs.git
                version: main
            src/ros2cs/custom_messages/autoware_msgs:
                type: git
                url: https://github.com/autowarefoundation/autoware/
                version: main
        ```

3. Now pull the repositories again (also the `custom_msgs` package repository)
    
    === "Ubuntu"
        ```bash
        cd ~/ros2-for-unity
        ./pull_repositories.sh
        ```

    <!-- === "Windows"
        ```powershell
        cd C:\ros2-for-unity
        .\pull_repositories.ps1
        ``` -->



### 3. Build ROS 2 For Unity


Build `ROS2ForUnity` with custom message packages using the following commands:

=== "Ubuntu"
    ```
    cd ~/ros2-for-unity
    ./build.sh --standalone
    ```

<!-- === "Windows"
    ```
    cd C:\ros2-for-unity
    .\build.ps1 -standalone
    ``` -->

### 4. Install `custom_msgs` to AWSIM

New `ROS2ForUnity` build, which you just made in step [3](#3-build-ros-2-for-unity), contains multiple libraries that already exist in the *AWSIM*.<br>
To install `custom_msgs` and not copy all other unnecessary files, you should get the `custom_msgs` related libraries only.

You can find them in following directories and simply copy to the analogous directories in `AWSIM/Assets/Ros2ForUnity` folder, or use the script described [here](#automation-of-copying-message-files).
=== "Ubuntu"
    - `ros2-for-unity/install/asset/Ros2ForUnity/Plugins` which names matches `custom_msgs_*`
    - `ros2-for-unity/install/asset/Ros2ForUnity/Plugins/Linux/x86_64/` which names matches `libcustom_msgs_*`
    

<!-- === "Windows"
      - `ros2-for-unity/install/asset/Ros2ForUnity/Plugins` which names matches `custom_msgs_*`
      - `ros2-for-unity/install/asset/Ros2ForUnity/Plugins/Windows/x86_64/` which names matches `custom_msgs_*` -->


#### Automation of copying message files
=== "Ubuntu"
    To automate the process, you can use a script that copies all files related to your `custom_msgs` package.

    1. Create a file named `copy_custom_msgs.sh` in directory `~/ros2-for-unity/` and paste the following content into it.
    ```bash
    #!/bin/bash
    echo "CUSTOM_MSGS_PACKAGE_NAME: $1"
    echo "AWSIM_DIR_PATH: $2"
    find ./install/asset/Ros2ForUnity/Plugins -maxdepth 1 -name "$1*"    -type f -exec cp {} $2/Assets/Ros2ForUnity/Plugins \;
    find ./install/asset/Ros2ForUnity/Plugins/Linux/x86_64 -maxdepth 1 -name     "lib$1*" -type f -exec cp {} $2/Assets/Ros2ForUnity/Plugins/Linux/x86_64 \;
    ```
    2. Save the file and give it executable rights with the command:
    ```bash
    chmod a+x copy_msgs.sh
    ```
    3. Run the script with two arguments:
    ```bash
    ./copy_custom_msgs.sh <CUSTOM_MSGS_PACKAGE_NAME> <AWSIM_DIR_PATH>
    ```

    - `<CUSTOM_MSGS_PACKAGE_NAME>` - the first one which is the name of the package with messages - in this case `custom_msgs`,
    - `<AWSIM_DIR_PATH>` - the second which is the path to the cloned *AWSIM* repository.

    !!! example
        ```bash
        ./copy_custom_msgs.sh custom_msgs ~/unity/AWSIM/
        ```
<!-- 
=== "Windows"
    To automate the process, you can use these commands with changed:
    
    - `<CUSTOM_MSGS_PACKAGE_NAME>` - the name of your package with messages - in this case `custom_msgs`,
    - `<AWSIM_DIR_PATH>` - to path to the cloned *AWSIM* repository
    ```powershell
    Get-ChildItem C:\ros2-for-unity\install\asset\Ros2ForUnity\Plugins\* -Include @('<CUSTOM_MSGS_PACKAGE_NAME>*') | Copy-Item -Destination <AWSIM_DIR_PATH>\Assets\Ros2ForUnity\Plugins
    Get-ChildItem C:\ros2-for-unity\install\asset\Ros2ForUnity\Plugins\Windows\x86_64\* -Include @('<CUSTOM_MSGS_PACKAGE_NAME>*') | Copy-Item -Destination <AWSIM_DIR_PATH>\Assets\Ros2ForUnity\Plugins\Windows\x86_64
    ```
    !!! example
        ```powershell
        Get-ChildItem C:\ros2-for-unity\install\asset\Ros2ForUnity\Plugins\* -Include @('custom_msgs*') | Copy-Item -Destination C:\unity\AWSIM\Assets\Ros2ForUnity\Plugins
        Get-ChildItem C:\ros2-for-unity\install\asset\Ros2ForUnity\Plugins\Windows\x86_64\* -Include @('custom_msgs*') | Copy-Item -Destination C:\unity\AWSIM\Assets\Ros2ForUnity\Plugins\Windows\x86_64
        ``` -->

### 5. Test
Make sure that the package files `custom_msgs` have been properly copied to the `AWSIM/Assets/Ros2ForUnity`.<br>
Then try to create a message object and check in the console of `Unity Editor` if it compiles without errors.

Taking `Header` as an example, the simplest way to create an object is:
```
var topic = new std_msgs.msg.Header();
topic.Frame_id = "map";
topic.Stamp.Sec = 1234567;
```

`ROS2` messages in Unity are just a structure containing the same fields - keep the same names and types.<br>
It is same as `C#` structure to access its fields for reading and filling.

!!!warning
    The first letter of each message field in `Unity` is always uppercase - even if it is described with lowercase in `ROS2` message.
