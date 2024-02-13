## Introduction

`ClockPublisher` allows the publication of the simulation time from the clock operating within AWSIM. The current time is retrived from a `TimeSource` object via the `SimulatorROS2Node`. The AWSIM provides convenient methods for selecting the appropriate time source type, as well as the flexibility to implement custom `TimeSources` tailored to specific user requirements.

## Setup

To enable the publication of the current time during simulation execution, `ClockPublisher` must be included as a component within the scene. Moreover, to allow the `TimeSource` to be set or changed, the `TimeSourceSelector` object must also be present in the active scene.

![time_source_selector_hierarchy](time_source_selector_hierarchy.png)

#### Selecting Time Source

The desired `TimeSource` can be selected in two ways:

- **Inspector Selection:** `TimeSource` type can be conveniently choosen directly within the editor interface.

![time_source_selector_inspector](time_source_selector_inspector.png)

- **JSON Configuration File:** Alternatively, the `TimeSource` type can be specified in the JSON configuration file via the _TimeSource_ field. The value for this field can be found in the [list of available time sources](#list-of-time-sources) in the _JSON File_ column.

![time_source_selector_config](time_source_selector_config.png)


#### List of Time Sources

| Inspector Type | JSON File | Description |
|:-|:-|:-|
| UNITY | unity | based on the time of the _Unity Engine_ |
| DOTNET | system | based on system time, starting with time since epoch, progressing according to simulation timescale |
| DOTNET_SIMULATION | simulation | based on system time, starting with zero value, progressing according to simulation timescale |
| SS2 | ss2 | driven by an external source, used by the [scenario simulator v2](../../ScenarioSimulation/PreparingTheConnectionBetweenAWSIMAndScenarioSimulator/) |



## Architecture

The `ClockPublisher` operates within a dedicated thread called the _'Clock'_ thread. This design choice offers significant advantages, as it frees the publishing process from the constraints imposed by [fixed update limits](../../ROS2/ROS2ForUnity/index.md#upper-limit-to-publish-rate). As a result, `ClockPublisher` is able to consistently publish time at a high rate, ensuring stability and accuracy.

### Accessing Time Source

Running the clock publisher in a dedicated thread introduced the challenge of accessing shared resources by different threads. In our case, the _Main Thread_ and _Clock Thread_ compete for `TimeSoruce` resources. The diagram below illustrates this concurrent behavior, with two distinct threads vying for access to the `TimeSource`:

- **Main Thread**: included publishing message by sensors, (on the diagram overlap by blueish region labeled sensor loop)
- **Clock Thread**: included clock publisher, (in the diagram blueish region labeled clock loop)

Given multiple sensors, each with its own publishing frequency, alongside a clock running at 100Hz, there is a notable competition for `TimeSource` resources. In such cases, it becomes imperative for the `TimeSource` class to be thread-safe.

![clock_publisher_threads](clock_publisher_threads.png)


### Thread-Safe Time Source

The `TimeSource` synchronization mechanism employs a mutex to lock the necessary resource for the current thread. The sequence of actions undertaken each time the `GetTime()` method is called involves:

- acquiring the lock,
- getting the current time, e.g. system time since epoch, _(this may differ for other types of time sources)_,
- obtaining the current simulation time-scale,
- calculating the delta time since previous call, influenced by the time scale,
- returning the current time,
- releasing the lock.

![time_source_mutex](time_source_mutex.png)


### Extensions

There are two additional classes used to synchronise the _UnityEngine_ `Time` and `TimeScale` values between threads:

- `TimeScaleProvider`: facilitates the synchronisation of the time scale value of the simulation across threads,
- `UnityTimeProvider`: provides access to _UnityEngine_ `Time` and `TimeAsDouble` to the threads other than the main thread.