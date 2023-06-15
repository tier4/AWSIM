# NPC Vehicle

`NPCVehicle` is a non-playable object that simulates a vehicle that is stationary or moving around the scene. It can move on roads, more specifically `TrafficLanes`, thanks to the use of `RandomTrafficSimulator` - which you can read more about [here](../../RandomTrafficSimulator/). Vehicles moving on the scene take into account each other - avoiding collisions, follow traffic lights and have an implemented mechanism of yielding the right of way.

![vehicles](vehicles.png)

!!! note "Sample scene"
    If you would like to see how `NPCVehicle` works or run some tests, we encourage you to familiarize yourself with the `NPCVehicleSample` default scene described in this [section](../../../DefaultExistingScenes/).

!!! tip "Ego Vehicle"
    If you are interested in the most important vehicle on the scene - `Ego Vehicle`, we encourage you to read this [section](../../EgoVehicle/).

#### Prefabs and Fbxs

Prefabs can be found under the following path:
<br>`AWSIM\Assets\AWSIM\Prefabs\NPCs\Vehicles\*`<br>
The table shows the available prefabs of the vehicles:


|              |        Hatchback         |         SmallCar         |           Taxi           |          Truck           |           Van            |
| :----------: | :----------------------: | :----------------------: | :----------------------: | :----------------------: | :----------------------: |
| *Appearance* | ![vehicles](image_1.png) | ![vehicles](image_2.png) | ![vehicles](image_3.png) | ![vehicles](image_4.png) | ![vehicles](image_5.png) |
|   *Prefab*   |    `Hatchback.prefab`    |    `SmallCar.prefab`     |     `Taxi-64.prefab`     |    `Truck_2t.prefab`     |       `Van.prefab`       |

`NPCVehicle` prefab has the following content:<br>
![prefab_link](prefab_link.png)

As you can see, it consists of 2 parents for *GameObjects*: `Visuals` - aggregating visual elements, `Colliders` - aggregating colliders and single object `CoM`.  All objects are described in the sections below.



#### Visual elements

Prefabs are developed using models available in the form of `*.fbx` files.
For each vehicle, the visuals elements and `LOD` were loaded from the appropriate `*.fbx` file.
The `LOD` is always added as components of the main-parent *GameObject* in prefab, while the visual elements of the model are aggregated and added in object `Visuals`.

`*.fbx` file for each vehicle is located in the appropriate `Models` directory for the vehicle under the following path:<br>
`Assets/AWSIM/Models/NPCs/Vehicles/<vehicle_name>/Models/<vehicle_name>.fbx`

As you can see, the additional visual element is `Driver`.<br>
![driver](driver.png)<br>
It was also loaded from the `*.fbx` file which can be found under the following path:<br>
`AWSIM/Assets/AWSIM/Models/NPCs/Vehicles/Driver/Model/Driver.fbx`

!!! example "Vehicle fbx"
    The content of a sample `*.fbx` file is presented below, all elements except `Collider` have been added to the prefab as visual elements of the vehicle.
     `Collider` is used as the *Mesh* source for the `Mesh Collider` in the `BodyCollider` object.<br>
    ![fbx](fbx.png).


#### Link
The default scene does not have vehicles implemented in fixed places, but they are spawned by `RandomTrafficSimulator` which is located in the `Environment` prefab. Therefore, before starting the simulation, no `NPCVehicle` object is on the scene.

When you run the simulation, you can see objects appearing as children of `RandomTrafficSimulator`:<br>
![traffic_link](traffic_link.png)

In each `NPCVehicle` prefab, the local coordinate system of the vehicle (main prefab link) should be defined in the axis of the rear wheels projected onto the ground - in the middle of the distance between them. This aspect holds significance when characterizing the dynamics of the object, as it provides convenience in terms of describing its motion and control.

<img src=link_2.png width=500px> <img src=link.png width=500px>

#### Components and Scripts
![prefab](prefab.png)

There are several components responsible for the full functionality of `NPCVehicle`:

- *[LOD Group](https://docs.unity3d.com/Manual/class-LODGroup.html)* - provides level of detail configuration for shaders - affects *GPU* usage.
- *[Rightbody](https://docs.unity3d.com/ScriptReference/Rigidbody.html)* - ensures that the object is controlled by the physics engine in *Unity* - e.g. pulled downward by gravity.
- *NPCVehicle Script* - provides the ability to change the position and orientation of the vehicle, as well as to control the turn signals and brake light.

Script can be found under the following path:<br>
`AWSIM\Assets\AWSIM\Scripts\NPCs\Vehicles` 


## CoM
`CoM` (*Center of Mass*) is an additional link that is defined to set the center of mass in the `Rightbody`. The *NPC Vehicle Script* is responsible for its assignment. This measure should be defined in accordance with reality. Most often, the center of mass of the vehicle is located in its center, at the height of its wheel axis - as shown below.

<img src=com_2.png width=500px> <img src=com.png width=500px>


## Colliders
*Colliders* are used to ensure collision between objects. In `NPCVehicle`, the main `BodyCollider` collider and `Wheels Colliders` colliders for each wheel were added.

### Body Collider
![collider](collider.png)

`BodyCollider` is a vehicle object responsible for ensuring collision with other objects, in addition, it can be used to detect these collisions. The `MeshCollider` takes a *Mesh* of object and builds its `Collider`.
based on that it. The *Mesh* for the `BodyCollider` was also loaded from the `*.fbx` file similarly to the visual elements.

![collider_example](collider_example.png)
### Wheels Colliders
![wheel_collider](wheel_collider.png)

`WheelsColliders` are an essential element from the point of view of driving vehicles on the road, they are the only ones that have contact with the roads and it is important that they are properly configured.
Each vehicle, apart from the visual elements related to the wheels, should also have 4 colliders - for each wheel.

To prevent inspector entry for `WheelCollider`. `WheelColliderConfig` has been developed which ensures that friction is set to 0 and only wheel suspension and collisions are enabled.

<!-- TODO: Radius, Suspension, Suspension Spring -->

![wheel_collider_example](wheel_collider_example.png)

!!! tip "Wheel Collider Config"
    For a better understanding of the meaning of `WheelCollider` we encourage you to read [this manual](https://docs.unity3d.com/Manual/class-WheelCollider.html).


## LOD

![lod](lod.png)

`LOD` provides dependence of the level of detail of the object depending on the ratio of the *GameObject’s* screen space height to the total screen height. Vehicle models have only one `LOD0` group, therefore there is no reduction in model complexity when it does not occupy a large part of the screen. It is only culled when it occupies less than 2% of the height.

## Rightbody

![rightbody](rightbody.png)

`RightBody` ensures that the object is controlled by the physics engine. The `Mass` of the vehicle should approximate its actual weight. In order for the vehicle to physically interact with other objects - react to collisions, `Is Kinematic` must be turned off. The `Use Gravity` should be turned on - to ensure the correct behavior of the body during movement. In addition, `Interpolate` should be turned on to ensure the physics engine's effects are smoothed out.

## NPC Vehicle Script
![script](script.png)

The script takes the `Rightbody` and provides an inputs that allows the `NPCVehicle` to move. 
Script inputs give the ability to set the position and orientation of the vehicle, taking into account the effects of suspension and gravity. In addition, the script uses the [`CoM`](#com) link reference to assign the center of mass of the vehicle to the `Rightbody`.

Script inputs are used by `RandomTrafficSimulator`, which controls the vehicles on the scene - it is described [here](../../../Components/Environment/TrafficComponents/).

#### Input Data

| Category      | Type    | Description                                                                                                                                                                                             |
| :------------ | :------ | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| *SetPosition* | Vector3 | Move the `NPCVehicle` so that its x, z coodinates are same as the specified coordinates. Pitch and roll are determined by physical operations that take effects of suspension and gravity into account. |
| *SetRotation* | Vector3 | Rotate the `NPCVehicle` so that its yaw becomes equal to the specified one. Vertical movement is determined by physical operations that take effects of suspension and gravity into account.            |


`Visual Object Root` is a reference to the parent aggregating visuals, it can be used to disable the appearance of visual elements of the `NPCVehicle` in the scene.

Whereas `Bounds` Represents an axis aligned bounding box of the `NPCVehicle`. It is used primarily to detect collisions between vehicles in the event of spawning, yielding and others. Moreover, vehicle bounds are displayed by [*Gizmos*](https://docs.unity3d.com/ScriptReference/Gizmos.html).<br>
<img src=bounds_example.png width=500px>

The settings of the remaining elements, i.e. the `Axle` and the `Lights`, are described [here](#axle-settings) and [here](#lights-settings).

!!! question
    If you don't see *Gizmo's* visual elements, remember to turn them on.<br>
    ![gizmo](gizmo.png)


### Axle Settings
![script_axle](script_axle.png)

This part of the settings is responsible for the proper connection of visual elements with the collider for each wheel - described earlier. The objects configured in this section are used to control the vehicle - its wheel speed and steering angle, which are calculated based on the input values. Correct configuration is very important from the point of view of the possibility of `NPCVehicle` movement on the road.

### Lights Settings
![script_axle](script_lights.png)

This part of the settings is related to the configuration of the emission of materials when a specific lighting is activated. There are 3 types of lights: `Brake`, `Left Turn Signal` and `Right Turn Signal`.Each of the lights has its visual equivalent in the form of a *Mesh*, in the case of `NPCVehicle` all lights are in the *Mesh* of the `Body` object, which has many materials - including those related to lights. 

For each type of light, the appropriate `Material Index` (equivalent of element index in mesh) and `Lighting Color` are assigned - yellow for `Turn Signals`, red for `Break`. 

`Lighting Intensity` values are also configured - the greater the value, the more light will be emitted. This value is related to `Lighting Exposure Weight` parameter that is a exposure weight - the lower the value, the more light is emitted.

The brake light is switched on depending on the speed of the `NPCVehicle`, while `RandomTrafficSimulator` is responsible for switching on and off the turn signal.

![materials](materials.png)
