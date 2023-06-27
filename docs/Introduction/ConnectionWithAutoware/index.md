
<video width="1920" controls autoplay muted loop>
<source src="awsim_video.mp4" type="video/mp4">
</video>

[*Autoware*](../Autoware/) is a powerful open-source software platform for autonomous driving. Its modular architecture, including perception, localization, planning, and control modules, provides a comprehensive framework for developing self-driving vehicles. [*Autoware*](../Autoware/) combined with  [*AWSIM*](../AWSIM/) simulator provides safe testing, validation, and optimization of autonomous driving algorithms in diverse scenarios.

!!! note "Run with Autoware"
    If you would like to know how to run AWSIM with Autoware, we encourage you to read this [section](../../UserGuide/Installation/RunDemo/).


## Combination Architecture
![](awsim_autoware.png)

The combination of *AWSIM* with *Autoware* is possible thanks to *Vehicle Interface* and *Sensing* modules of [*Autoware*](../Autoware/) architecture. The component responsible for ensuring connection with these modules from the *AWSIM* side is `EgoVehicle`. It has been adapted to the *Autoware* architecture and provides *ROS2* topic-based communication. However, the other essential component is `ClockPublisher`, which provides simulation time for *Autoware* - also published on the topic - more details [here](../../UserGuide/ProjectGuide/Components/Scene/).

`EgeVehicle` component provides the publication of the current vehicle status through a script working within `Vehicle Status`. It provides real-time information such as: current speed, current steering of the wheels or current states of lights - these are outputs from *AWSIM*. 

On the other hand, `Vehicle Ros Input` is responsible for providing the values of the outputs from *Autoware*. It subscribes to the current commands related to the given acceleration, gearbox gear or control of the specified lights.  

Execution of the received commands is possible thanks to `Vehicle`, which ensures the setting of appropriate accelerations on the `**Wheel` and controlling the visual elements of the vehicle.

The remaining data delivered from *AWSIM* to *Autoware* are sensors data, which provides information about the current state of the surrounding environment and those necessary to accurately estimate `EgoVehicle` position.

More about `EgoVehicle` and its scripts is described in this [section](../../UserGuide/ProjectGuide/Components/EgoVehicle/).

### Sequence diagram
Below is a simplified sequential diagram of information exchange in connection between *AWSIM* and *Autoware*. As you can see, the first essential information published from *AWSIM* is `Clock` - the simulation time. Next, `EgoVehicle` is spawned and first sensors data are published, which are used in the process of automatic position initialization on *Autoware* side. At the same time, the simulation on *AWSIM* side is updated.

Next in the diagram is the main information update loop in which:

- During each cycle there is a synchronization with the time from the simulation.
- *AWSIM* publishes data from sensors available in `EgoVehicle`, which are taken into account in the processes carried out in *Autoware*.
- The control commands from *Autoware* are subscribed by *AWSIM*, which are executed on *AWSIM* side and `EgoVehicle` update is performed.
- The current state of the `EgoVehicle` is published.

The order of information exchange presented in the diagram is a simplification. The exchange of information takes place through the publish-subscribe model and each data is sent with a predefined frequency.

![](autoware_awsim_sequence.png)


## Features

#### Engagement
Autoware in combination with AWSIM can participate in common road situations

- Drive straight in lane on an intersection
![go straight](straight_green.gif)
- Turn on an intersection
![turn](turn_green.gif)

#### Traffic light recognition
AWSIM allows Autoware to recognize traffic lights and act accordingly

- Stop at a red light
![stop on red](stop_red.gif)
- Drive at green light
![run on green](straight_green.gif)
- Still drive at a yellow light
![run on yellow](straight_yellow.gif)

#### Interaction with vehicles
<!-- TODO -->

#### Interaction with pedestrians
<!-- TODO -->




!!! node "draft"

    - Engagement (driving straight, turning - with a view of the intersection shape, **gifs**)
    - Traffic light recognition
        - Stopping at a red and yellow lights (**gifs**)
        - Running a green and yellow lights (**gifs**)
    - Interaction with vehicles
        - Following (with suddenly stop, **gifs**)
        - Right-of-way at the intersection (turning right, forcing->stopping, **gifs**)
        - Cut-in situation (**gifs**)
    - Interaction with pedestrians
        - Right-of-way at a crosswalk (with red light and forcing, **gifs**)
        - Pedestrian on the road beyond the crosswalk (**gifs**)
    - Detecting bad behaviors (**Future**, **gifs)**
    <!-- TODO everything -->
