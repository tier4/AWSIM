<!-- reusable hyperlinks -->
[qos]: https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html
[header]: https://docs.ros2.org/latest/api/std_msgs/msg/Header.html
[time]: https://docs.ros2.org/latest/api/builtin_interfaces/msg/Time.html

# ROS2 For Unity
[*Ros2ForUnity*](https://github.com/RobotecAI/ros2-for-unity) (`R2FU`) module is a communication solution that effectively connects *Unity* and the *ROS2* ecosystem, maintaining a strong integration.
Unlike other solutions, it doesn't rely on bridging communication but rather utilizes the *ROS2* middleware stack (specifically the `rcl` layer and below), enabling the inclusion of *ROS2* nodes within *Unity* simulations.

`R2FU` is used in *AWSIM* for many reasons.
First of all, because it offers high-performance integration between *Unity* and *ROS2*, with improved throughput and lower latencies compared to bridging solutions.
It provides real *ROS2* functionality for simulation entities in *Unity*, supports standard and custom messages, and includes convenient abstractions and tools, all wrapped as a *Unity* asset.
For a detailed description, please see [*README*](https://github.com/RobotecAI/ros2-for-unity/blob/master/README.md).

## Prerequisites
This asset can be prepared in two flavours:

- *standalone mode* - where no *ROS2* installation is required on target machine, e.g., your *Unity* simulation server.
All required dependencies are installed and can be used e.g. as a complete set of *Unity* plugins.
- *overlay mode* - where the *ROS2* installation is required on target machine.
Only asset libraries and generated messages are installed therefore *ROS2* instance must be sourced.

By default, asset `R2FU` in *AWSIM* is prepared in *standalone mode*.

!!! warning

    To avoid internal conflicts between the standalone libraries, and sourced ones, *ROS2* instance shouldn't be sourced before running AWSIM or the *Unity* Editor.

!!! question "Can't see topics"
    There are no errors but I can't see topics published by `R2FU`

    - Make sure your DDS ([Localhost settings](../../../GettingStarted/QuickStartDemo/#localhost-settings)) config is correct.
    - Sometimes *ROS2* daemon brakes up when changing network interfaces or *ROS2* version.
Try to stop it forcefully (`pkill -9 ros2_daemon`) and restart (`ros2 daemon start`).

## Concept
Describing the concept of using `R2FU` in *AWSIM*, we distinguish:

- *ROS2Node* - it is the equivalent of a node in *ROS2*, it has its own name, it can have any number of subscribers, publishers, service servers and service clients.
In the current *AWSIM* implementation, there is only one main node.
- *ROS2Clock* - it is responsible for generating the simulation time using the selected source.
- *SimulatorROS2Node* - it is a class that is directly responsible for *AWSIM<->ROS2* communication, and contains one instance each of *ROS2Node* and *ROS2Clock*, so it creates the main *AWSIM* node in *ROS2* and simulates the time from the selected source (currently [UnityTimeSource](https://docs.unity3d.com/ScriptReference/Time.html) is used).
It is initialized at the moment of running the scene in *Unity* - thanks to the [`RuntimeInitializeOnLoadMethod`](https://docs.unity3d.com/ScriptReference/RuntimeInitializeOnLoadMethodAttribute.html) mark.
- *Publisher* - it is the equivalent of the publisher in *ROS2*, it uses a single topic on which it can publish a selected type of message, and it has a selected [*QoS*][qos] profile.
Each publisher in *AWSIM* is created in *ROS2Node* object of class *SimulatorROS2Node*.
- *Subscriber* - it is the equivalent of the subscriber in *ROS2*, it uses a single topic from which it subscribes to the selected type of message, and it has a selected [*QoS*][qos] profile.
Each subscriber in *AWSIM* is created in *ROS2Node* object of class *SimulatorROS2Node*.
 
<!-- - TODO: *Service Server* -  -->
<!-- - TODO: *Service Client* - -->

The *SimulatorROS2Node* implementation, thanks to the use of `R2FU`, allows you to add communication via *ROS2* to any Unity component.
For example, we can receive control commands from any other *ROS2* node and publish the current state of *Ego*, such as its position in the environment.

!!! tip "Simulation time"
    If you want to use system time (*ROS2* time) instead of *Unity* time, use `ROS2TimeSource` instead of `UnityTimeSource` in the `SimulatorROS2Node` class.

## Package structure
`Ros2ForUnity` asset contains:

- *Plugins* - dynamically loaded libraries for *Windows* and *Linux* (`*.dll` and `*.so` files).
In addition to the necessary libraries, here are the libraries created as a result of generation the types of *ROS2* messages that are used in communication.
- *Scripts* - scripts for using `R2FU`  in *Unity* - details [below](#scripts).
- *Extension Scripts* - scripts for using `R2FU` in *AWSIM*, provide abstractions of a single main *Node* and simplify the interface  - details [below](#extension-scripts).
These scripts are not in the library itself, but directly in the directory `Assets/AWSIM/Scripts/ROS/**`.

### Scripts
- `ROS2UnityCore` - the principal class for handling *ROS2* nodes and executables.
Spins and executes actions (e.g. clock, sensor publish triggers) in a dedicated thread.
- `ROS2UnityComponent` - `ROS2UnityCore` adapted to work as a *Unity* component.
- `ROS2Node` - a class representing a *ROS2* node, it should be constructed through `ROS2UnityComponent` class, which also handles spinning.
- `ROS2ForUnity` - an internal class responsible for handling checking, proper initialization and shutdown of *ROS2* communication,
- `ROS2ListenerExample` - an example class provided for testing of basic *ROS2->Unity* communication.
- `ROS2TalkerExample` - an example class provided for testing of basic *Unity->ROS2* communication.
- `ROS2PerformanceTest` - an example class provided for performance testing of *ROS2<->Unity* communication.
- `Sensor` - an abstract base class for *ROS2*-enabled sensor.
- `Transformations` - a set of transformation functions between coordinate systems of *Unity* and *ROS2*.
- `PostInstall` - an internal class responsible for installing `R2FU` metadata files.
- `Time` scripts - a set of classes that provide  the ability to use different time sources:
    - `ROS2Clock` - *ROS2* clock class that for interfacing between a time source (*Unity* or *ROS2* system time) and *ROS2* messages.
    - `ROS2TimeSource` - acquires *ROS2* time (system time by default).
    - `UnityTimeSource` - acquires *Unity* time.
    - `DotnetTimeSource` - acquires *Unity* `DateTime` based clock that has resolution increased using `Stopwatch`.
    - `ITimeSource` - interface for general time extraction from any source.
    - `TimeUtils` - utils for time conversion.

### Extension Scripts
Additionally, in order to adapt *AWSIM* to the use of `R2FU`, the following scripts are used:

- `SimulatorROS2Node` - it is a class that is directly responsible for *AWSIM<->ROS2* communication.
- `ClockPublisher` - allows the publication of the simulation time from the clock running in the *SimulatorROS2Node*.
It must be added as a component to the scene in order to publish the current time when the scene is run.

    ![clock_publisher](clock_publisher.png)

- `QoSSettings` - it is the equivalent of *ROS2* [*QoS*][qos], which allows to specify the *QoS* for subscribers and publishers in *AWSIM*.
It uses the [`QualityOfServiceProfile`](https://github.com/RobotecAI/ros2cs/blob/develop/src/ros2cs/ros2cs_core/QualityOfServiceProfile.cs) implementation from the [Ros2cs](https://github.com/RobotecAI/ros2cs) library.
- `ROS2Utility` - it is a class with utils that allow, for example, to convert positions in the *ROS2* coordinate system to the *AWSIM* coordinate system.
- `DiagnosticsManager` - prints diagnostics for desired elements described in `*.yaml` config file.

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
- [autoware_auto_msgs](https://github.com/tier4/autoware_auto_msgs):
    - [`autoware_auto_control_msgs`](https://index.ros.org/p/autoware_auto_control_msgs/gitlab-autowarefoundation-autoware-auto-autoware_auto_msgs/#humble),
    - [`autoware_auto_geometry_msgs`](https://index.ros.org/p/autoware_auto_geometry_msgs/gitlab-autowarefoundation-autoware-auto-autoware_auto_msgs/#humble),
    - [`autoware_auto_planning_msgs`](https://index.ros.org/p/autoware_auto_planning_msgs/gitlab-autowarefoundation-autoware-auto-autoware_auto_msgs/#humble),
    - [`autoware_auto_mapping_msgs`](https://index.ros.org/p/autoware_auto_mapping_msgs/gitlab-autowarefoundation-autoware-auto-autoware_auto_msgs/#humble),
    - [`autoware_auto_vehicle_msgs`](https://index.ros.org/p/autoware_auto_vehicle_msgs/gitlab-autowarefoundation-autoware-auto-autoware_auto_msgs/#humble).
- [tier4_autoware_msgs](https://github.com/tier4/tier4_autoware_msgs):
    - [`tier4_control_msgs`](https://github.com/tier4/tier4_autoware_msgs/tree/tier4/universe/tier4_control_msgs),
    - [`tier4_vehicle_msgs`](https://github.com/tier4/tier4_autoware_msgs/tree/tier4/universe/tier4_vehicle_msgs).
- Others:
    - [`tf2_msgs`](https://index.ros.org/p/tf2_msgs/github-ros2-geometry2/#humble),
    - [`unique_identifier_msgs`](https://index.ros.org/p/unique_identifier_msgs/github-ros2-unique_identifier_msgs/#humble).

In order for the message package to be used in *Unity*, its `*.dll` and `*.so` libraries must be generated using `R2FU`.

!!! tip "Custom message"
    If you want to generate a custom message to allow it to be used in *AWSIM* please read [this tutorial](../AddACustomROS2Message/).

## Use of generated messages in *Unity*
Each message type is composed of other types - which can also be a complex type.
All of them are based on built-in [*C#* types](https://learn.microsoft.com/en-us/dotnet/csharp/language-reference/builtin-types/built-in-types).
The most common built-in types in messages are `bool`, `int`, `double` and `string`.
These types have their communication equivalents using *ROS2*.

A good example of a complex type that is added to other complex types in order to specify a reference - in the form of a timestamp and a frame - is [std_msgs/Header][header].
This message has the following form:

```csharp
builtin_interfaces/msg/Time stamp
string frame_id
```

!!! warning "ROS2 directive"
    In order to work with *ROS2* in *Unity*, remember to add the directive `using ROS2;` at the top of the file to import types from this namespace.
    
### Create an object
The simplest way to create an object of [`Header`][header] type is:

```csharp
var header = new std_msgs.msg.Header()
{
    Frame_id = "map"
}
```

It is not required to define the value of each field.
As you can see, it creates an object, filling only `frame_id` field - and left the field of complex [`builtin_interfaces/msg/Time`][time] type initialized by default.
Time is an important element of any message, how to fill it is written [here](#filling-a-time).

### Accessing and filling in message fields 
As you might have noticed in the previous example, a *ROS2* message in *Unity* is just a structure containing the same fields - keep the same names and types.
Access to its fields for reading and filling is the same as for any *C#* structure.

```csharp
var header2 = new std_msgs.msg.Header();
header2.Frame_id = "map";
header2.Stamp.sec = "1234567";
Debug.Log($"StampSec: {header2.Stamp.sec} and Frame: {header2.Frame_id}");
```

!!! warning "Field names"
    There is one always-present difference in field names.
    The **first letter** of each message field in *Unity* is **always** **uppercase** - even if the base *ROS2* message from which it is generated is lowercase.

### Filling a time
In order to complete the time field of the [`Header`][header] message, we recommend the following methods in *AWSIM*:

1. When the message has no [`Header`][header] but only the [`Time`][time] type:

    ```csharp
    var header2 = new std_msgs.msg.Header();
    header2.Stamp = SimulatorROS2Node.GetCurrentRosTime();
    ```

2. When the message has a [`Header`][header] - like for example  [autoware_auto_vehicle_msgs/VelocityReport](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/VelocityReport.idl):

    ```csharp
    velocityReportMsg = new autoware_auto_vehicle_msgs.msg.VelocityReport()
    {
        Header = new std_msgs.msg.Header()
        {
            Frame_id = "map",
        }
    };
    var velocityReportMsgHeader = velocityReportMsg as MessageWithHeader;
    SimulatorROS2Node.UpdateROSTimestamp(ref velocityReportMsgHeader);
    ```

These methods allow to fill the [`Time`][time] field in the message object with the simulation time - from *ROS2Clock*

### Create a message with array
Some message types contain an [array](https://learn.microsoft.com/en-us/dotnet/csharp/programming-guide/arrays/) of some type.
An example of such a message is [`nav_msgs/Path`](https://docs.ros2.org/foxy/api/nav_msgs/msg/Path.html), which has a [`PoseStamped`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html) array.
In order to fill such an array, you must first create a [`List<T>`](https://learn.microsoft.com/pl-pl/dotnet/api/system.collections.generic.list-1?view=net-8.0), fill it and then convert it to a [raw array](https://learn.microsoft.com/pl-pl/dotnet/api/system.collections.generic.list-1.toarray?view=net-8.0).

```csharp
var posesList = new List<geometry_msgs.msg.PoseStamped>();
for(int i=0; i<=5;++i)
{
    var poseStampedMsg = new geometry_msgs.msg.PoseStamped();
    poseStampedMsg.Pose.Position.X = i;
    poseStampedMsg.Pose.Position.Y = 5-i;
    var poseStampedMsgHeader = poseStampedMsg as MessageWithHeader;
    SimulatorROS2Node.UpdateROSTimestamp(ref poseStampedMsgHeader);
    posesList.Add(poseStampedMsg);
}
var pathMsg = new nav_msgs.msg.Path(){Poses=posesList.ToArray()};
var pathMsgHeader = pathMsg as MessageWithHeader;
SimulatorROS2Node.UpdateROSTimestamp(ref pathMsgHeader);
// pathMsg is ready
```

### Publish on the topic
In order to publish messages, a publisher object must be created.
The static method `CreatePublisher` of the `SimulatorROS2Node` makes it easy.
You must specify the *type* of message, the *topic* on which it will be published and the *QoS* profile.<br>
Below is an example of `autoware_auto_vehicle_msgs.msg.VelocityReport` type message publication with a frequency of `30Hz` on `/vehicle/status/velocity_status` topic, the [*QoS*][qos] profile is `(Reliability=Reliable, Durability=Volatile, History=Keep last, Depth=1`):

```csharp
using UnityEngine;
using ROS2;

namespace AWSIM
{
    public class VehicleReportRos2Publisher : MonoBehaviour
    {
        float timer = 0;
        int publishHz = 30;
        QoSSettings qosSettings = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 1,
        };
        string velocityReportTopic = "/vehicle/status/velocity_status";
        autoware_auto_vehicle_msgs.msg.VelocityReport velocityReportMsg;
        IPublisher<autoware_auto_vehicle_msgs.msg.VelocityReport> velocityReportPublisher;

        void Start()
        {
            // Create a message object and fill in the constant fields
            velocityReportMsg = new autoware_auto_vehicle_msgs.msg.VelocityReport()
            {
                Header = new std_msgs.msg.Header()
                {
                    Frame_id = "map",
                }
            };

            // Create publisher with specific topic and QoS profile
            velocityReportPublisher = SimulatorROS2Node.CreatePublisher<autoware_auto_vehicle_msgs.msg.VelocityReport>(velocityReportTopic, qosSettings.GetQoSProfile());
        }

         bool NeedToPublish()
        {
            timer += Time.deltaTime;
            var interval = 1.0f / publishHz;
            interval -= 0.00001f;
            if (timer < interval)
                return false;
            timer = 0;
            return true;
        }

        void FixedUpdate()
        {
            // Provide publications with a given frequency
            if (NeedToPublish())
            {
                // Fill in non-constant fields
                velocityReportMsg.Longitudinal_velocity = 1.00f;
                velocityReportMsg.Lateral_velocity = 0.00f;
                velocityReportMsg.Heading_rate = 0.00f;

                // Update Stamp
                var velocityReportMsgHeader = velocityReportMsg as MessageWithHeader;
                SimulatorROS2Node.UpdateROSTimestamp(ref velocityReportMsgHeader);

                // Publish
                velocityReportPublisher.Publish(velocityReportMsg);
            }
        }
    }
}
```

#### Upper limit to publish rate

The above example demonstrates the implementation of the `'publish'` method within the `FixedUpdate` Unity event method. However, this approach has certain limitations. The maximum output frequency is directly tied to the current value of `Fixed TimeStep` specified in the `Project Settings`. Considering that the AWSIM is targeting a frame rate of 60 frames per second (FPS), the current `Fixed TimeStep` is set to 1/60s. And this impose 60Hz as a limitation on the publish rate for any sensor, which is implemented within `FixedUpdate` method. In case a higher output frequency be necessary, an alternative implementation must be considered or adjustments made to the `Fixed TimeStep` setting in the `Editor->Project Settings->Time`.

The table provided below presents a list of sensors along with examples of topics that are constrained by the `Fixed TimeStep` limitation.

| Object | Topic |
|:-|:-|
| GNSS Sensor | /sensing/gnss/pose |
| IMU Sensor | /sensing/imu/tamagawa/imu_raw |
| Traffic Camera | /sensing/camera/traffic_light/image_raw |
| Pose Sensor | /awsim/ground_truth/vehicle/pose |
| OdometrySensor | /awsim/ground_truth/localization/kinematic_state |
| LIDAR | /sensing/lidar/top/pointcloud_raw |
| Vehicle Status | /vehicle/status/velocity_status |

If the sensor or any other publishing object within AWSIM does not have any direct correlation with physics (i.e., does not require synchronization with physics), it can be implemented without using the `FixedUpdate` method. Consequently, this allows the bypass of upper limits imposed by the `Fixed TimeStep`.

The table presented below shows a list of objects that are not constrained by the `Fixed TimeStep` limitation.

| Object | Topic |
|:-|:-|
| Clock | /clock |

### Subscribe to the topic
In order to subscribe messages, a subscriber object must be created.
The static method `CreateSubscription` of the `SimulatorROS2Node` makes it easy.
You must specify the *type* of message, the *topic* from which it will be subscribed and the [*QoS*][qos] profile.
In addition, the *callback* must be defined, which will be called when the message is received - in particular, it can be defined as a [lambda expression](https://learn.microsoft.com/en-us/dotnet/csharp/language-reference/operators/lambda-expressions).<br>
Below is an example of `std_msgs.msg.Bool` type message subscription on `/vehicle/is_vehicle_stopped` topic, the [*QoS*][qos] profile is [`“system default”`][qos]:
```csharp
using UnityEngine;
using ROS2;

namespace AWSIM
{
    public class VehicleStoppedSubscriber : MonoBehaviour
    {
        QoSSettings qosSettings = new QoSSettings();
        string isVehicleStoppedTopic = "/vehicle/is_vehicle_stopped";
        bool isVehicleStopped = false;
        ISubscription<std_msgs.msg.Bool> isVehicleStoppedSubscriber;

        void Start()
        {
            isVehicleStoppedSubscriber = SimulatorROS2Node.CreateSubscription<std_msgs.msg.Bool>(isVehicleStoppedTopic, VehicleStoppedCallback, qosSettings.GetQoSProfile());
        }

        void VehicleStoppedCallback(std_msgs.msg.Bool msg)
        {
            isVehicleStopped = msg.Data;
        }

        void OnDestroy()
        {
            SimulatorROS2Node.RemoveSubscription<std_msgs.msg.Bool>(isVehicleStoppedSubscriber);
        }
    }
}
```
<!-- TODO: Service Server -  -->
<!-- TODO: Service Client - -->
