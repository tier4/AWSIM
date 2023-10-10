#AWSIM Environment

## Introduction
`Environment` is an object that contains all the elements visible on the scene along with components that affect how they are rendered.
It contains several objects aggregating static environment objects in terms of their type.
Moreover, it contains elements responsible for controlling random traffic.

![environment](environment.png)

!!! tip "Own Environment prefab"
    If you would like to develop your own prefab `Environment` for *AWSIM*, we encourage you to read this [tutorial](../../Environment/AddNewEnvironment/AddEnvironment/).

!!! note "AutowareSimulation scene"
    If you would like to see how `Environment` with random traffic works or run some tests, we encourage you to familiarize yourself with the `AutowareSimulation` scene described in this [section](../../../ProjectGuide/Scenes/#autowaresimulation).

Prefab `Environment` is also used to create a point cloud (`*.pcd` file) needed to locate the `EgoVehicle` in the simulated *AWSIM* scene.
The point cloud is created using the [`RGL`](https://github.com/RobotecAI/RobotecGPULidar) plugin and then used in *Autoware*.
We encourage you to familiarize yourself with an example scene of creating a point cloud - described [here](../../../ProjectGuide/Scenes/#pointcloudmapping).

!!! tip "Create PointCloud (*.pcd file)"
    If you would like to learn how to create a point cloud in *AWSIM* using `Environment` prefab, we encourage you to read this [tutorial](../../Environment/CreatePCD/).

### Architecture
The architecture of an `Environment` - with dependencies between components - is presented on the following diagram.

![environment diagram](../../../Introduction/AWSIM/environment.png)

### Prefabs

Prefabs can be found under the following path: 

| Name                          | Description                                              | Path                                                                   |
| :---------------------------- | :------------------------------------------------------- | :--------------------------------------------------------------------- |
| *Nishishinjuku*               | Only stationary visual elements, no traffic              | `Assets/AWSIM/Prefabs/Environments/Nishishinjuku.prefab`               |
| *Nishishinjuku RandomTraffic* | Stationary visual elements along with random traffic     | `Assets/AWSIM/Prefabs/Environments/Nishishinjuku RandomTraffic.prefab` |
| *Nishishinjuku Traffic*       | Stationary visual elements along with non-random traffic | `Assets/AWSIM/Prefabs/Environments/Nishishinjuku Traffic.prefab`       |

!!! note "Environment prefab" 
    Due to the similarity of the above prefabs, this section focuses on prefab `Nishishinjuku RandomTraffic`.
    The exact differences between `Nishishinjuku RandomTraffic` and `Nishishinjuku Traffic` will be described in the future.

!!! note "Environment name"
    In order to standardize the documentation, the name `Environment` will be used in this section as the equivalent of the prefab named `Nishishinjuku RandomTraffic`.

`Nishishinjuku RandomTraffic` prefab has the following content:

![environment_link](environment_link.png)

As you can see it contains:

- `SJK*` objects - which are aggregators for visual models.
- `RandomTrafficSimulator`, `TrafficIntersections`, `TrafficLanes`, `StopLines` - which are responsible for random traffic of [`NPCVehicles`](../../Traffic/NPCs/Vehicle/).
- `NPCPedestrians` - which is an aggregator of [`NPCPedestrian`](../../Traffic/NPCs/Pedestrian/) prefabs added to the scene.
- `Volume`, `Directional Light` - which are components that affect the appearance of objects on the scene.

All of these objects are described below in this section.

### Visual elements
`Nishishinjuku RandomTraffic` prefab contains many visual elements which are described [here](#visual-elements-sjk).

### Link in the default Scene
`Nishishinjuku RandomTraffic` prefab is added to the `Environment` object - between which there is rotation about the `Oy` axis by 90 degrees.
This rotation is added because of the differences in coordinate alignments between the `Nishishinjuku RandomTraffic` prefab objects (which have been modeled as `*.fbx` files) and the specifics of the *GridZone* definition (more on this is described [here](#components)).

Object `Environment` is added to `AutowareSimulation` which is added directly to the main parent of the scene - there are no transformations between these objects.

![environment_link_2](environment_link_2.png)

### Components
![environment_prefab](environment_prefab.png)

`Nishishinjuku RandomTraffic` (`Environment`) prefab contains only one component:

 - [*Environment* (script)](#environment-script) - which is important for communicating with *Autoware* and loading elements from *Lanelet2*.
Because it allows to define the location of the environment in relation to the world.


### Layers
In order to enable the movement of vehicles around the environment, additional layers have been added to the project: `Ground` and `Vehicle`.

![layers](layers.png)

All objects that are acting as a ground for `NPCVehicles` and `EgoVehicle` to move on have been added to `Ground` layer - they cannot pass through each other and should collide for the physics engine to calculate their interactions.

![model_layer](model_layer.png)

For this purpose, `NPCVehicles` and `EgoVehicle` have been added to the `Vehicle` layer.

![vehicle_layer](vehicle_layer.png)

In the project physics settings, it is ensured that collisions between objects in the `Vehicle` layer are disabled (this applies to `EgoVehicle` and `NPCVehicles` - they do not collide with each other):

![layers_physis](layers_physis.png)

## Traffic Components
Due to the specificity of the use of `RandomTrafficSimulator`, `TrafficIntersections`, `TrafficLanes`, `StopLines` objects, they have been described in a separate section [*Traffic Components*](../../Traffic/TrafficComponents/) - where all the elements necessary in simulated random traffic are presented.

## Visual Elements (SJK)
The visuals elements have been loaded and organized using the `*.fbx` files which can be found under the path: 

```
Assets/AWSIM/Externals/Nishishinjuku/Nishishinjuku_optimized/Models/*
```

`Environment` prefab contains several objects aggregating stationary visual elements of space by their category:

- `SJK01_P01` - contains all objects constituting the ground of the environment, these are roads and green fields - each of them contains a `MeshColliders` and layer set as `Ground` to ensure collisions with `NPCVehicles` and `EgoVehicle`.

    ![sjk1](sjk1.png) 

- `SJK01_P02` - contains all road surface markings on roads added to the environment.
The objects of this group do not have `MeshColliders` and their layer is `Default`.

    ![sjk2](sjk2.png)

- `SJK01_P03` - contains all the vertical poles added to the environment, such as lamp posts, road signs and traffic light poles.
Only `TrafficLight` poles and `PedestrianLight` poles have `MeshCollider` added.
The layer for all objects is `Default`.

    ![sjk3](sjk3.png)

- `SJK01_P04` - contains all barriers added to the environment, such as barriers by sidewalks.
The objects of this group do not have `MeshColliders` and their layer is `Default`.

    ![sjk4](sjk4.png)

- `SJK01_P05` - contains all greenery added to the environment, such as trees, shrubs, fragments of greenery next to buildings.
The objects of this group do not have `MeshColliders` and their layer is `Default`.

    ![sjk5](sjk5.png)

- `SJK01_P06` - contains all buildings added to the environment.
Objects of this category also have a `MeshCollider` added, but their layer is `Default`.

    ![sjk6](sjk6.png)

!!! warning "Scene Manager"
    For models (visual elements) added to the prefab to work properly with the [`LidarSensor`](../../Sensors/LiDARSensor/LiDARSensor/) sensor using [`RGL`](../../Sensors/LiDARSensor/RGLUnityPlugin/), make sure that the `SceneManager` component is added to the scene - more about it is described in this [section](../../Sensors/LiDARSensor/RGLUnityPlugin/#scenemanager).

    In the scene containing `Nishishinjuku RandomTraffic`prefab *Scene Manager* (script) is added as a component to the `AutowareSimulation` object containing the `Environment`.

    ![scene_manager](scene_manager.png)

### TrafficLights
`TrafficLights` are a stationary visual element belonging to the `SJK01_P03` group.
The lights are divided into two types, the classic `TrafficLights` used by vehicles at intersections and the `PedestrianLights` found at crosswalks.

![sjk_lights_link](lights/sjk_lights_link.png)

Classic traffic lights are aggregated at object `TrafficLightA01_Root01_ALL_GP01`

![pedestrian_light_link](lights/lights_link.png)

while lights used by pedestrians are aggregated at object `TrafficLightB01_Root01_All_GP01`.

![pedestrian_light_link](lights/pedestian_lights_link.png)

`TrafficLights` and `PedestrianLights` are developed using models available in the form of `*.fbx` files, which can be found under the following path:<br>
`Assets/AWSIM/Externals/Nishishinjuku/Nishishinjuku_opimized/Models/TrafficLights/Models/*`

#### Classic TrafficLights
![light](lights/light.png) 

`TrafficLights` lights, outside their housing, always contain 3 signaling light sources of different colors  - from left to right: green, yellow, red.
Optionally, they can have additional sources of signaling the ability to drive in a specific direction in the form of one or three signaling arrows.

![light_prefab](lights/light_prefab.png)

In the environment there are many classic lights with different signaling configurations.
However, each contains:

- *Transform* - defines the position of the lights in the environment relative to the main parent of the group (`SJK01_P03`).
- *Mesh Filter* - contains a reference to the `Mesh` of the object.
- *Mesh Renderer* - enables the rendering of `Mesh`, including its geometry, textures, and materials, giving it a visual appearance in the scene.
- *Mesh Collider* - allows an object to have collision detection based on `Mesh`.
- [*Traffic Light* (script)](../../Traffic/TrafficComponents/#traffic-light-script) - provides an interface to control signaling by changing the emission of materials.
This script is used for simulated traffic, so it is described.

##### Materials
An important element that is configured in the `TrafficLights` object are the materials in the `Mesh Renderer` component.
Material with index 0 always applies to the housing of the lights.
Subsequent elements 1-6 correspond to successive slots of light sources (round luminous objects) - starting from the upper left corner of the object in the right direction, to the bottom and back to the left corner.
These indexes are used in script *Traffic Light* (script) - described [here](../../Traffic/TrafficComponents/#traffic-light-script).

![lights_mesh](lights/lights_materials.png)

Materials for lighting slots that are assigned in `Mesh Renderer` can be found in the following path:
`Assets/AWSIM/Externals/Nishishinjuku/Nishishinjuku_opimized/Models/TrafficLights/Materials/*`


#### PedestrianLights
![pedestrian_light](lights/pedestrian_light.png)

`PedestrianLights` lights, outside their housing, always contain 2 signaling light sources of different colors - red on top and green on the bottom.

![pedestrian_lights_prefab](lights/pedestrian_lights_prefab.png)

In the environment there are many pedestrian lights - they have the same components as classic `TrafficLights`, but the main difference is the configuration of their materials.

##### Materials
An important element that is configured in the `PedestrianLights` object are the materials in the `Mesh Renderer` component.
Material with index 0 always applies to the housing of the lights.
Subsequent elements 1-2 correspond to successive slots of light sources (round luminous objects) - starting from top to bottom.
These indexes are used in script *Traffic Light* (script) - described [here](../../Traffic/TrafficComponents/#traffic-light-script).

![pedestrian_lights_mesh](lights/pedestrian_lights_materials.png)<br>
Materials for lighting slots that are assigned in `Mesh Renderer` can be found in the following path:
`Assets/AWSIM/Externals/Nishishinjuku/Nishishinjuku_opimized/Models/TrafficLights/Materials/*`

## Volume
![volume](volume.png)

`Volume` is *GameObject* with [*Volume*](https://docs.unity3d.com/Packages/com.unity.render-pipelines.high-definition@11.0/manual/Volumes.html) component which is used in the [*High Definition Render Pipeline*](https://docs.unity3d.com/Packages/com.unity.render-pipelines.high-definition@10.2/manual/index.html) (*HDRP*).
It defines a set of scene settings and properties.
It can be either global, affecting the entire scene, or local, influencing specific areas within the scene.
*Volumes* are used to interpolate between different property values based on the Camera's position, allowing for dynamic changes to environment settings such as fog color, density, and other visual effects.

In case of prefab `Nishishinjuku RandomTraffic` volume works in global mode and has loaded [*Volume profile*](https://docs.unity3d.com/Packages/com.unity.render-pipelines.high-definition@11.0/manual/Volume-Profile.html).
This volume profile has a structure that overrides the default properties of *Volume* related to the following components: [Fog](https://docs.unity3d.com/Packages/com.unity.render-pipelines.high-definition@11.0/manual/Override-Fog.html), [Shadows](https://docs.unity3d.com/Packages/com.unity.render-pipelines.high-definition@11.0/manual/Override-Shadows.html), [Ambient Occlusion](https://docs.unity3d.com/Packages/com.unity.render-pipelines.high-definition@11.0/manual/Override-Ambient-Occlusion.html), [Visual Environment](https://docs.unity3d.com/Packages/com.unity.render-pipelines.high-definition@11.0/manual/Override-Visual-Environment.html), [HDRI Sky](https://docs.unity3d.com/Packages/com.unity.render-pipelines.high-definition@11.0/manual/Override-HDRI-Sky.html).
It can be found in the following path:<br>
`Assets/AWSIM/Prefabs/Environments/Nishishinjuku/Volume Profile.asset`

## Directional Light
![directional_light_prefab](directional_light_prefab.png)

`Directional Light` is *GameObject* with [`Light`](https://docs.unity3d.com/Packages/com.unity.render-pipelines.high-definition@12.0/manual/Light-Component.html) component which is used in the [*High Definition Render Pipeline*](https://docs.unity3d.com/Packages/com.unity.render-pipelines.high-definition@10.2/manual/index.html) (*HDRP*).
It controls the shape, color, and intensity of the light.
It also controls whether or not the light casts shadows in scene, as well as more advanced settings.


In case of prefab `Nishishinjuku RandomTraffic` a `Directional` type light is added.
It creates effects that are similar to sunlight in scene.
This light illuminates all *GameObjects* in the scene as if the light rays are parallel and always from the same direction.
`Directional` light disregards the distance between the Light itself and the target, so the light does not diminish with distance.
The strength of the Light (`Intensity`) is set to `73123.09 Lux`.
In addition, a `Shadow Map` with a resolution of `4096` is enabled, which is updated in `Every Frame` of the simulation.
The transform of the `Directional Light` object is set in such a way that it shines on the environment almost vertically from above.

![directional_light](directional_light.png)

## NPCPedestrians
`NPCPedestrians` is an aggregating object for `NPCPedestrian` objects placed in the environment.
Prefab `Nishishinjuku RandomTraffic` has 7 `NPCPedestrian` (`humanElegant`) prefabs defined in selected places.
More about this `NPCPedestrian` prefab you can read in this [section](../../../Components/Traffic/NPCs/Pedestrian/).

## Environment (script)
<!-- reusable link to MGRS -->
[mgrs]: https://www.maptools.com/tutorials/mgrs/quick_guide

![environment_script](environment_script.png)

*Environment* (script) contains the information about how a simulated `Environment` is positioned in real world.
That means it describes what is the real world position of a simulated `Environment`.

*AWSIM* uses part of a [*Military Grid Reference System*][mgrs] (*MGRS*).
To understand this topic, you only need to know, that using *MGRS* you can specify distinct parts of the globe with different accuracy.
For *AWSIM* the chosen accuracy is a 100x100 km square.
Such a square is identified with a unique code like `54SUE` (for more information on *Grid Zone* please see [this page][mgrs]).

Inside this *Grid Zone* the exact location is specified with the offset calculated from the bottom-left corner of the *Grid Zone*.
You can interpret the *Grid Zone* as a local coordinate system in which you position the `Environment`.

In the `Nishishinjuku RandomTraffic` prefab, the simulated `Environment` is positioned in the *Grid Zone* `54SUE`.
The offset if equal to `81655.73` meters in the `Ox` axis, `50137.43` meters in the `Oy` axis and `42.49998` meters in the `Oz` axis.
In addition to this shift, it is also necessary to rotate the Environment in the scene by `90` degrees about the `Oy` axis - this is ensured by the transform in the prefab object.

This means that the *3D* models were created in reference to this exact point and because of that the *3D* models of `Environment` align perfectly with the data from *Lanelet2*.

!!! warning "The essence of Environment (script)"
    The *Environment* (script) configuration is necessary at the moment of loading data from *Lanelet2*.

    Internally it shifts the elements from *Lanelet2* by the given offset so that they align with the `Environment` that is located at the local origin with no offset.
