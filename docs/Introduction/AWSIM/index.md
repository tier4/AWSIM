# AWSIM
<video width="1920" controls autoplay muted loop>
<source src="awsim_video.mp4" type="video/mp4">
</video>
[*AWSIM*](https://github.com/tier4/AWSIM) is an open-source simulator made with [*Unity*](https://unity.com/) for autonomous driving research and development. 
It is developed for self-driving software like [*Autoware*](../Autoware/). This simulator aims to bridge the gap between the virtual and real worlds, enabling users to train and evaluate their autonomous systems in a safe and controlled environment before deploying them on real vehicles. It provides a realistic virtual environment for training, testing, and evaluating various aspects of autonomous driving systems. 

*AWSIM* simulates a variety of real-world scenarios, with accurate physics and sensor models. It offers a wide range of sensors, such as: *Cameras*, *GNSS*, *IMU* and  *LiDARs*, allowing developers to simulate their  autonomous vehicle's interactions with the environment accurately. The simulator also models dynamic objects, such as pedestrians, other vehicles, and traffic lights, making it possible to study interactions and decision-making in complex traffic scenarios. This enables the testing and evaluation of perception, planning, and control algorithms under different sensor configurations and scenarios.

*AWSIM* supports a flexible and modular architecture, making it easy to customize and extend its capabilities. Users can modify the current or add a new environment with their own assets and traffic rules to create custom scenarios to suit their specific research needs. This allows for the development and testing of advanced algorithms in diverse driving conditions.

Because *AWSIM* was developed mainly to work with [*Autoware*](../Autoware/), it supports:

- *Ubuntu 22.04* and *Windows 10/11*
- [*ROS2*](https://docs.ros.org/en/humble/index.html) Humble distribution

!!! note "Prerequisites"
    You can read more about the prerequisites and running *AWSIM* [here](../../GettingStarted/QuickStartDemo/).


!!! note "Connection with Autoware"
    Introduction about how the connection between *AWSIM* and *Autoware* works can be read [here](../CombinationWithAutoware/).

#### Why was AWSIM developed?

The main objectives of *AWSIM* are to facilitate research and development in autonomous driving, enable benchmarking of algorithms and systems, and foster collaboration and knowledge exchange within the autonomous driving community. By providing a realistic and accessible platform, *AWSIM* aims to accelerate the progress and innovation in the field of autonomous driving.

## Architecture
![](awsim.png)

To describe the architecture of *AWSIM*, first of all, it is necessary to mention the `Scene`. It contains all the objects occurring in the simulation of a specific scenario and their configurations. The default *AWSIM* scene that is developed to work with [*Autoware*](https://github.com/autowarefoundation/autoware) is called *AutowareSimulation*.

In the scene we can distinguish basics components such like `MainCamera`, `ClockPublisher`, `EventSystem` and `Canvas`. A detailed description of the scene and its components can be found [here](../../ProjectGuide/Scenes/). 

Besides the elements mentioned above, the scene contains two more, very important and complex components: `Environment` and `EgoVehicle` - described below.

#### Environment
![](environment.png)

`Environment` is a component that contains all `Visual Elements` that simulate the environment in the scene and those that provide control over them. It also contains two components `Directional Light` and `Volume`, which ensure suitable lighting for `Visual Elements` and simulate weather conditions. A detailed description of these components can be found [here](../../Components/Environment/AddNewEnvironment/AddEnvironment/#create-an-environment-prefab).

In addition to `Visual Elements` such as buildings or greenery, it contains the entire architecture responsible for traffic. The traffic involves `NPCVehicles` that are spawned in the simulation by `TrafficSimulator` - using traffic components. A quick overview of the traffic components is provided below, however, you can read their detailed description [here](../../Components/Traffic/TrafficComponents/).

`NPCPedestrians` are also `Environment` components, but they are not controlled by `TrafficSimulator`. They have added scripts that control their movement - you can read more details [here](../../Components/Traffic/NPCs/Pedestrian/).

##### Traffic Components

`TrafficLanes` and `StopLines` are elements loaded into `Environment` from *Lanelet2*.
`TrafficLanes` have defined cross-references in such a way as to create routes along the traffic lanes. In addition, each `TrafficLane` present at the intersection has specific conditions for yielding priority. `TrafficSimulator` uses `TrafficLanes` to spawn `NPCVehicles` and ensure their movement along these lanes. If some `TrafficLanes` ends just before the intersection, then it has a reference to `StopLine`. Each `StopLine` at the intersection with `TrafficLights` has reference to the nearest `TrafficLight`. `TrafficLights` belong to one of the visual element groups and provide an interface to control visual elements that simulate traffic light sources (bulbs). A single `TrafficIntersection` is responsible for controlling all `TrafficLights` at one intersection.
Detailed description of mentioned components is in [this section](../../Components/Traffic/TrafficComponents/).


#### EgoVehicle
![](egovehicle.png)

`EgoVehicle` is a component responsible for simulating an autonomous vehicle moving around the scene. It includes:

- `Models` and `Reflection Probe` components related to its visual appearance. 
- `Colliders` providing collisions and the ability to move on roads.
- `Sensors` providing data related to the state of the vehicle, including its position and speed in `Environment` and the state of its surroundings.
- `Vehicle` component that simulates dynamics, controls `**Wheel` and is responsible for ensuring their movement.
- `Vehicle Ros Input` and `Vehicle Keyboard Input `components that have a reference to the `Vehicle` object and set control commands in it.
- `Vehicle Visual Effect` provides an interface for `Vehicle` to control the lighting.

A detailed description of `EgoVehicle` and its components mentioned above can be found [here](../../Components/Vehicle/EgoVehicle/). The sensor placement on `EgoVehicle` used in the default scene is described [here](../../Components/Vehicle/URDFAndSensors/). Details about each of the individual sensors are available in the following sections: [`Pose`](../../Components/Vehicle/URDFAndSensors/#pose), [`GNSS`](../../Components/Sensors/GNSSSensor/), [`LiDAR`](../../Components/Sensors/LiDARSensor/LiDARSensor/), [`IMU`](../../Components/Sensors/IMUSensor/), [`Camera`](../../Components/Sensors/CameraSensor/), [`Vehicle Status`](../../Components/Sensors/VehicleStatusSensor/).


#### FixedUpdate Limitation

In AWSIM, the sensors' publishing methods are triggered from the `FixedUpdate` function and the output frequency is controlled by:

```csharp
time += Time.deltaTime;
var interval = 1.0f / OutputHz;
interval -= 0.00001f; // Allow for accuracy errors.
if (time < interval)
    return;
timer = 0;
```

Since this code runs within the FixedUpdate method, it's essential to note that `Time.deltaTime` is equal to `Fixed Timestep`, as stated in the [Unity Time.deltaTime documentation](https://docs.unity3d.com/ScriptReference/Time-deltaTime.html). Consequently, with each invocation of FixedUpdate, the `time` variable in the sensor script will increment by a constant value of `Fixed Timestep`, independent of the actual passage of real-time. Additionally, as outlined in the [Unity documentation](https://docs.unity3d.com/Manual/ExecutionOrder.html), the `FixedUpdate` method might execute multiple times before the `Update` method is called, resulting in extremely small time intervals between successive FixedUpdate calls. The diagram below illustrates the mechanism of invoking the FixedUpdate event function."

![](fixed_update_execution.jpg)

During each frame (game tick) following actions are performed:

- first, the `Delta Time` is calculated as the difference between the current frame and the previous frame,
- next, the `Delta Time` is added to the `Time` (regular time),
- afterward, a check is made to determine how much `Fixed Time` (physics time) is behind the `Time`,
- if the difference between `Time` and `Fixed Time` is equal to or greater then `Fixed Timestep`, the `Fixed Update` event function is invoked,
- if `FixedUpdate` function were called, the `Fixed Timestep` is added to the `Fixed Time`,
- once again, a check is performed to assess how much `Fixed Time` is behind the `Time`,
- <strong>if necessary, the `FixedUpdate` function is called again,</strong>
- if the difference between the `Time` and the `Fixed Time` is smaller than the `Fixed Timestep`, the `Update` method is called, followed be scene rendering and other Unity event functions.

As a consequence, this engine feature may result in unexpected behavior when FPS (Frames Per Second) are unstable or under certain combinations of FPS, Fixed Timestep, and sensor OutputHz

In case of low frame rates, it is advisable to reduce the `Time Scale` of the simulation. The Time Scale value impacts simulation time, which refers to the time that is simulated within the model and might or might not progress at the same rate as real-time. Therefore, by reducing the time scale, the progression of simulation time slows down, allowing the simulation more time to perform its tasks.
