<!-- TODO remaining -->

!!! Draft-note
    (highlight the part required by ego - not needed in npc)

    - Create a prefab (**gif**)
    - Add a Rightbody (**gif**)
    - Add visual elements (Body, Wheels, Sensor Kit) (what is needed, how to divide the model, how to import, **gifs**)
    - Add a center of mass (how to estimate, **screen**)
    - Add a reflection probe (sample configuration, **screen**)
    - Add a vehicle collider (what is needed, sample configuration, **screen**)
    - Add wheel colliders (how to properly configure, sample **screen**)
    - Add a base for sensors (URDF) (where to define, **screen**)
    - Add a Vehicle Keyboard Input Script (input limits)
    - Add a Vehicle Visual Effect (brake, turn signal, reverse lights, how to test - **gifs**)
    - Add a Vehicle Script (axes and physics settings, input limits, how to test - **video**)
    - Add links for sensors (how to define - reference to *sensors_calibration*, difference in right-handed and left-handed coordinate systems, **screens**)
    - Adding sensors
        - Add a VehicleStatusSensor (frame_id, Autoware topics, qos, how to test - **screens**)
        - Add a LiDAR (scene manager - hyperlink to 7.1.1, model selection, frame_id, Autoware topics, qos, visualization, ranges, gaussian noise, how to test - **screen**)
        - Add a IMU (frame_id, Autoware topics, qos, how to test - **screens**)
        - Add a GNSS (frame_id, Autoware topics, qos, how to test - **screens**)
        - Add a Camera (camera preview, fov, how prepare distorsion shader, ros image shader, parameters, frame_id, Autoware topics, qos, how to test - **screens** - including traffic light recognition)
    - Add a Vehicle Ros Input Script (disable keyboard input, Autoware topics, qos, how to test - hyperlink)
    - Add a Vehicle to scene (hyperlink)

!!! info "Ego Vehicle Component"
    In this tutorial we will create a new Ego Vehicle.
    To learn more about what an Ego Vehicle is in AWSIM please visit [Ego Vehicle description page](../../../UserGuide/ProjectGuide/Components/Vehicle/).

## Cerate an Object
Add a child *Object* to the *Simulation* called `EgoVehicle`.

![ego vehicle add object](ego_vehicle_add_object.gif)

## Add a Rigidbody
1. While having a newly created *Ego Vehicle* *Object* selected, in the *Inspector* view click on the 'Add Component' button, search for `Rigidbody` and select it.

    ![rigidbody add component](rigidbody_add_component.gif)

    ![rigidbody search](rigidbody_search.png)

1. Configure Mass and Drag with the correct values for your Vehicle.

    ![rigidbody configure 1](rigidbody_configure1.gif)

1. Configure Interpolation and Collision Detection.

    ![rigidbody configure 2](rigidbody_configure2.gif)

## Add visual elements
For a detailed explanation hwo to add visual elements of your Vehicle check out this [dedicated tutorial](./AddVisualElements/).

## Add a Canter of Mass
To add a center of mass to your vehicle you have to add a `CoM` child *Object* to the `EgoVehicle` *Object* (the same as in steps before).

Then just set the *position* of the `CoM` *Object* in the *Inspector* view to represent real-world center of mass of the Vehicle.

![center of mass](com_all.gif)

??? question "How to get to know the center of mass of my Vehicle"
    <!-- TODO: add guidelines on how to estimate CoM -->

## Add a Reflection Probe
1. Add a new *Object* called `Reflection Probe` as a child to the `EgoVehicle` *Object*.

    ![reflection probe add object](reflection_probe_add_object.gif)

2. Click on the 'Add Component' button, in the windows that pops-up search for `Reflection Probe` and select it.

    ![reflection probe add component](reflection_probe_add_component.gif)

    !!!note
        Please note that with `Reflection Probe` there should also be automatically added a ``HD Additional Reflection Data` Script.

        ![reflection probe additional script](reflection_probe_additional_script.png)

3. Configure the `Reflection Probe` as you wish.

    !!! example "Example Configuration"
        Below you can see an example configuration of the `Reflection Probe`.

        ![reflection probe configuration](reflection_probe_configuration.png)

## Add Colliders
For a detailed explanation how to add colliders to your Vehicle check out this [dedicated tutorial](./AddColliders/).

## Add a base for sensors (URDF)
You will most certainly want to add some sensors to your Ego Vehicle.
First you need to create a parent *Object* for all those sensors called `URDF`.
To do this we will add a child *Object* `URDF` to the `EgoVehicle` *Object*.

![urdf add object](urdf_add_object.gif)

This *Object* will be used as a base for all sensors we will add later.

## Add a Vehicle Script
To be able to control your Ego Vehicle you need a `Vehicle` Script.

1. Add the `Vehicle` Script to the `EgoVehicle` *Object*.

    ![vehicle script add component](vehicle_script_add_component.gif)

    ![vehicle script search](vehicle_script_search.png)

1. Configure the `Vehicle` Script *Axle Settings* and *Center Of Mass Transform*.

    ![vehicle script configure](vehicle_script_configure.gif)

## Add a Vehicle Keyboard Input Script
If, for any reason, you would like to control your Ego Vehicle in the simulation by hand you can do that with just one Script called `Vehicle Keyboard Input`.

If you want to add it just click the 'Add Component' button on the `EgoVehicle` *Object* and search for `Vehicle Keyboard Input` Script and select it.

![vehicle keyboard input script add](vehicle_keyboard_input_script.gif)

## Add a Vehicle Visual Effect Script
For a visual indication of a Vehicle status you will need a `Vehicle Visual Effect` Script.
To add and configure it follow the steps below.

1. Add a `Vehicle Visual Effect` Script by clicking 'Add Component' button, searching for it and selecting it.

    ![vehicle visual effect script add component](vehicle_visual_effect_script_add_component.gif)

1. Configure the lights.

    !!!note
        In this step we will configure only `Brake Lights`, but should repeat this for every Light.
        The process is almost the same for all Lights - just change the mesh renderer and lighting settings according to your preference.

    ![vehicle visual effect script configure](vehicle_visual_effect_script_configure.gif)

## Add a Vehicle Ros Input Script
<!-- TODO -->

## Add Sensors
<!-- TODO -->

### AWSIM Sensors
<!-- TODO: ad links to sensors -->

## Add a Vehicle to Scene
<!-- TODO -->
