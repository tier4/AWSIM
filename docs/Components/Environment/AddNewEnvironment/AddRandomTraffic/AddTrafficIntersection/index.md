Every `TrafficIntersection` on the scene needs to be added as a *GameObject*.
Best practice is to create a parent object `TrafficIntersections` and add all instances of `TrafficIntersection` as its children.
You can do this the same as with [Random Traffic Simulator](../../../../../Components/Environment/AddNewEnvironment/AddRandomTraffic/AddRandomTraffic/).

!!! warning "Traffic Lights configuration"
    Before performing this step, check all `TrafficLights` for correct configuration and make sure that `TrafficLights` have added scripts. If you want to learn how to add and configure it check out [this tutorial](../../AddTrafficLights/).

## Add a Box Collider
1. `TrafficIntersection` needs to be marked with a box collider.
First click on the `Add Component` button.

    ![Add box collider gif](traffic_intersection_add_box.gif)

1. In the window that popped up search for `Box Collider` and select it.

    ![Search for box collider](box_collider_search.png)

1. Then set the position and orientation and size of the Box Collider.
You can do this by manipulating Box Collider properties `Center` and `Size` in the *Inspector* view.

    ![Set traffic intersection size](traffic_intersection_size.png)

    !!! info "Traffic Intersection Box Collider guidelines"
        When adding a Box Collider marking your Traffic Intersection please make sure that

        - It is **not** floating over the ground - there is no gap between the Box Collider and The Traffic Intersection
        - It is high enough to cover all Vehicles that will be passing through the Intersection
        - It accurately represents the shape, position and orientation of the Traffic Intersection

## Add a Traffic Intersection Script
1. Click on the `Add Component` button.

    ![Add traffic intersection script gif](traffic_intersection_add_script.gif)

1. In the window that popped up search for `Traffic Intersection` and select it.

    ![Search for traffic intersection script](traffic_intersection_search.png)

1. You need to set a proper Collider Mask in order for the script to work.

    ![Set collider mask](traffic_intersection_collider_mask.png)

### Create traffic light groups
Traffic Light Groups are groups of traffic lights that are synchronized, meaning they light up with the same color and pattern at all times.

Traffic lights are divided into groups to simplify the process of creating a lighting sequence.
By default you will see 4 Traffic Light Groups, you can add and remove them to suit your needs.

1. First choose from the drop-down menu called `Group` the Traffic Light Group name you want to assign to your Traffic Light Group.

    <!-- ![Traffic intersection add traffic light gif](traffic_intersection_add_traffic_light.gif) -->
    <img src="traffic_intersection_add_traffic_light.gif" alt="Traffic intersection add traffic light gif" width="600"/>

1. Then add as many Traffic Lights as you want your group to have.
From the drop-down menu select the Traffic Lights you want to add.

    <!-- ![Traffic intersection add traffic light 2 gif](traffic_intersection_add_traffic_light2.gif) -->
    <img src="traffic_intersection_add_traffic_light2.gif" alt="Traffic intersection add traffic light 2 gif" width="600"/>

    !!! tip "Select Traffic Lights visually"
        If you have a lot of Traffic Lights it can be challenging to add them from the list.
        You can select them visually from the Scene the same as you had selected Traffic Lanes in the [Random Traffic Simulator](../../../../../Components/Environment/AddNewEnvironment/AddRandomTraffic/AddRandomTraffic/#add-spawnable-lanes).

### Create lighting sequences
Lighting Sequences is a list of commands based on which the Traffic Lights will operate on an intersection.
The elements in the Lighting Sequences list are changes (commands) that will be executed on the Traffic Light Groups.

Group Lighting Order should be interpreted as a command (or order) given to all Traffic Lights in selected Traffic Light Group.
In Group Lighting Orders you can set different traffic light status for every Traffic Light Group (in separate elements).
Lighting sequences list is processed in an infinite loop.

It should be noted that changes applied to one Traffic Light Group will remain the same until the next Group Lighting Order is given to this Traffic Light Group.
This means that if in one Group Lighting Order no command is sent to a Traffic Light Group then this Group will remain its current lighting pattern (color, bulb and status).

For every Lighting Sequences Element you have to specify the following

1. Interval Sec

    This is the time for which the sequence should wait until executing next order, so how long this state will be active.

1. For every element in Group Lighting Orders there needs to be specified

    1. Group to which this order will be applied
    2. List of orders (Bulb Data)

        In other words - what bulbs should be turned on, their color and pattern.
        
        - Type - What type of bulb should be turned on
        - Color - What color this bulb should have (in most cases this will be the same as color of the bulb if specified)
        - Status - How the bulb should light up (selecting `SOLID_OFF` is necessary only when you want to turn the Traffic Light completely off, meaning **no** bulb will light up)

        !!!note
            When applying the change to a Traffic Light

            - First all bulbs are turned off
            - Only after that changes made in the order are applied

            This means it is only necessary to supply the data about what bulbs should be turned on.
            E.g. you don't have to turn off a red bulb when turning on the green one.

!!!warning
    The first Element in the Lighting Sequences (in most cases) should contain bulb data for every Traffic Light Group.
    Traffic Light Groups not specified in the first Element will not light up at the beginning of the scene.

#### Example
Lets consider the following lighting sequence element.

![Example Group Lighting Orders](traffic_intersecton_example_sequence_element.png)

In the Lighting Sequence Element 5 we tell all Traffic Lights in the *Vehicle Traffic Light Group 2* to light up their Green Bulb with the color Green and status Solid On which means that they will be turned on all the time.
We also implicitly tell them to turn all other Bulbs off.

In the same time we tell all Traffic Lights in the *Pedestrian Traffic Light Group 2* to do the very same thing.

This state will be active for the next 15 seconds, and after that Traffic Intersection will move to the next Element in the Sequence.

Now lets consider the following Lighting Sequences Element 6.

![Example Group Lighting Order 2](traffic_intersecton_example_sequence_element2.png)

Here we order the Traffic Lights in the *Pedestrian Traffic Light Group 2* to light up their Green Bulb with the color Green and status Flashing.
We also implicitly tell them to turn all other bulbs off, which were already off from the implicit change in Element 5, so this effectively does nothing.

Note that Lighting Sequences Element 6 has no orders for *Vehicle Traffic Light Group 2*.
This means that Traffic Lights in the *Vehicle Traffic Light Group 2* will hold on to their earlier orders.

This state will be active for 5 seconds, which means that Traffic Lights in the *Vehicle Traffic Light Group 2* will be lighting solid green for the total of 20 seconds.

## How to test
To test how your Traffic Intersection behaves simply run the Scene as shown [here](../../../../../GettingStarted/SetupUnityProject/#run-the-demo-in-editor) (but don't launch *Autoware*).
To take a better look at the Traffic Lights you can change to the *Scene* view by pressing `ctrl + 1` - now you can move the camera freely (to go back to the *Game* view simply press `ctrl + 2`).

As the time passes you can examine whether your Traffic Intersection is configured correctly.
