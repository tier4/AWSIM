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
Your Ego Vehicle needs many individual visual parts.
Below we will add all needed visual elements.

First in `EgoVehicle` *Object* add a child *Object* called `Models`.

![ego vehicle add models](ego_vehicle_add_models.gif)

Inside `Models` *Object* we will add all visual models of out Ego Vehicle.

### Add a Body
First you will need to add a Body of your Vehicle.
It will contain many parts, so first lets create a `Body` parent *Object*.

![body add object](body_add_object.gif)

Next we will need to add Car Body

1. Add a child *Object* `CarBody` to the `Body` *Object*.

    ![body car add object](body_car_add_object.gif)

1. To the `CarBody` *Object* add a Mesh Filter.

    Click on the 'Add Component' button, search for `Mesh Filter` and select it.

    ![body car add mesh filter](body_car_add_mesh_filter.gif)

    ![mesh filter add component](mesh_filter_search.png)

1. To the `CarBody` *Object* add a Mesh Renderer.

    Click on the 'Add Component' button, search for `Mesh Filter` and select it

    ![body car add mesh renderer](body_car_add_mesh_renderer.gif)

    ![mesh renderer add component](mesh_renderer_search.png)

1. Specify Materials.

    You need to specify what materials will be used for rendering your Ego Vehicle model.
    Do this by adding elements to the `Materials` list and selecting the materials you wish to use as shown below.

    ![mesh renderer add materials](mesh_renderer_add_materials.gif)

    Add as many materials as your model has sub-meshes.

    !!!tip
        When you add too many materials, meaning there will be no sub-meshes to apply these materials to, you will see this warning.
        In such a case please remove materials until this warning disappears.

        ![mesh renderer too many materials](mesh_renderer_too_many_materials.png)

### Add interactive Body parts
In this step we will add the following parts

- Break light
- Reverse Light
- Right Turn Signal
- Left Turn Signal
- (optional) other visual elements required specifically by your Vehicle.

!!!info
    It may seem like all of the elements above can be parts of the `Body` mesh, but it is important for these parts to be separate, because we need to be able to make them interactive (e.g. flashing turn signals).

    Other good reason for having different meshes for Vehicle parts is that you have a Vehicle model, but for the simulation you need to add e.g. a roof rack with sensors - which can be achieved by adding more meshes.

We will illustrate this step only for Break Light, but you should repeat this step of the tutorial for each element of the list above.

1. Add a child *Object* to the `Body` *Object*.

    ![break light add object](break_light_add_object.gif)

2. Add a Mesh Filter and select the mesh (the same as in [section before](#add-a-body)).

    ![break light add mesh filter](break_light_add_mesh_filter.gif)

    ![mesh filter search](mesh_filter_search.png)

3. Add a Mesh Renderer and select the materials (the same as in [section before](#add-a-body)).

    ![break light add mesh renderer](break_light_add_mesh_renderer.gif)

    ![mesh renderer search](mesh_renderer_search.png)

### Add Wheels
In this step we will add individual visuals for every wheel.
This process is very similar to [the one before](#add-interactive-body-parts).

1. Add a child *Object* to the `Models` *Object* called `WheelVisuals`.

    ![wheel visuals add object](wheel_visuals_add_object.gif)

!!!note
    In this tutorial we will add only one wheel, but you should repeat the step for all 4 wheels.
    That is, follow the instructions that follow this message for every wheel your Vehicle has.

    - Front Left Wheel
    - Front Right Wheel
    - Rear Left Wheel
    - Rear Right Wheel

1. Add a child *Object* to the `WheelVisuals` *Object* called `FrontLeftWheel`.

    ![front left wheel add object](front_left_wheel_add_object.gif)

2. Add a child *Object* to the `FrontLeftWheel` *Object* called `WheelFrontL`.
    This *Object* will contain the actual wheel part.

    ![wheel front l add object](wheel_front_l_add_object.gif)

3. Add a Mesh Filter and select the wheel mesh.

    ![wheel front l add mesh filter](wheel_front_l_add_mesh_filter_component.gif)

    ![wheel front l configure mesh filter](wheel_front_l_configure_mesh_filter_component.gif)

4. Add a Mesh Renderer and select the wheel materials.

    ![wheel front l add mesh renderer](wheel_front_l_add_mesh_renderer_component.gif)

    ![wheel front l configure mesh renderer 1](wheel_front_l_configure_mesh_renderer_component1.gif)

    ![wheel front l configure mesh renderer 2](wheel_front_l_configure_mesh_renderer_component2.gif)

5. Repeat the steps before to add Breaks.

    The same way you have added the `WheelFrontL` *Object* now add the `WheelFrontLBreaks`.
    Naturally you will have to adjust the mesh and materials used as they will be different for breaks that for the wheel.

    Your final break configuration should look similar to the one following.

    ![wheel front l breaks configured](wheel_front_l_breaks_configured.png)

6. Set the `FrontLeftWheel` parent *Object* transformation to position the wheel in correct place.

    ![front left wheel set position](front_left_wheel_set_position.gif)

!!! success "Successful configuration"
    If you have done everything right your `WheelVisuals` *Object* should look similar to the one following.

    ![wheel visuals configured](wheel_visuals_configured.png)

### Move the models
The last step to correctly configure Vehicle models is to shift them so that the `EgoVehicle` origin is in the center of **fixed** axis.

This means you need to shift the whole `Models` *Object* accordingly (change the *position* fields in *transformation*).

!!!tip
    Add a `dummy` *Object* as a child to the `EgoVehicle` *Object* (the same as in steps before) so it is located in the origin of the `EgoVehicle`.
    
    Now move `Models` around relative to the `dummy` - change *position* in the *Inspector* view.
    The `dummy` will help you see when the **fixed** axis (in case of the Lexus from example it is the rear axis) is aligned with origin of `EgoVehicle`.

    In the end delete the `dummy` *Object* as it is no longer needed.

    ![models shift w dummy](models_shift_w_dummy.gif)

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
        Please note that with `Reflection Probe` there should also be automatically added a ``HD Additional Reflection Data` Script

        ![reflection probe additional script](reflection_probe_additional_script.png)

3. Configure the `Reflection Probe` as you wish.

    !!! example "Example Configuration"
        Below you can see an example configuration of the `Reflection Probe`.

        ![reflection probe configuration](reflection_probe_configuration.png)

## Add Colliders
Next you need to add Colliders to your Vehicle.
To do this follow the steps below.

1. Add a child *Object* called `Colliders` to the `EgoVehicle` *Object*.

    ![colliders add object](colliders_add_object.gif)

1. Shift parent *Object* `Colliders` accordingly as in earlier steps where we [shifted `Models`](#move-the-models).

### Add a Vehicle Collider
1. Add a child *Object* `Collider` to the `Colliders` *Object*.

    ![collider_add_object](collider_add_object.gif)

1. Add a `Mesh Collider` component to the `Collider` *Object* by clicking on the 'Add Component' button in the *Inspector* view and searching for it.

    ![collider add mesh collider](collider_add_mesh_collider.gif)

1. Click on the arrow in mesh selection field and from the pop-up window select your collider mesh.
    Next click on the check-box called `Convex`, by now your collider mesh should be visible in the editor.

    ![collider configure](collider_configure1280.gif)

### Add Wheel Colliders
1. Add a child *Object* `Wheels` to the `Colliders` *Object*.

    ![wheels add object](wheels_add_object.gif)

!!!note
    In this tutorial we will add only one wheel collider, but you should repeat the step for all 4 wheels.
    That is, follow the instructions that follow this message for every wheel your Vehicle has.

    - Front Left Wheel
    - Front Right Wheel
    - Rear Left Wheel
    - Rear Right Wheel

1. Add a child *Object* `FrontLeftWheel` to the `Wheels` *Object*.

    ![front left wheel add object](front_left_wheel_add_object_colliders.gif)

2. Add a `Wheel Collider` component to the `FrontLeftWheel` *Object* by clicking 'Add Component' and searching for it.

    ![wheel collider add component](wheel_collider_add_component.gif)

3. Add a `Wheel` Script to the `FrontLeftWheel` *Object* by clicking 'Add Component' and searching for it.

    ![wheel_script_add_component](wheel_script_add_component.gif)

4. Drag `FrontLeftWheel` *Object* from the `WheelVisuals` to the `Wheel visual Transform` field.

    ![wheel script configure transform](wheel_script_configure_transform2.gif)

5. Add a `Wheel Collider Config` Script to the `FrontLeftWheel` *Object* by clicking 'Add Component' and searching for it.

    ![wheel collider config add script](wheel_collider_config_script_add_component.gif)

6. Configure the `Wheel Collider Config` Script so that the Vehicle behaves as you wish.

    ![wheel collider config configure](wheel_collider_config_script_configure.gif)

7. Set the Transform of `FrontLeftWheel` *Object* to match the visuals of your Vehicle.

    ![front left wheel set transform](front_left_wheel_set_transform.gif)

!!! success "Successful configuration"
    If you have done everything right your `Colliders` *Object* should look similar to the one following.

    ![colliders configured](colliders_configured.png)
