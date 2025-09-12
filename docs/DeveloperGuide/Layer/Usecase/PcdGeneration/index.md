# PcdGeneration

## Abstract
`Pcd Generation` is a tool for a vehicle based point cloud mapping in a simulation environment.<br>
It is useful when you need a point cloud based on some location, but are not able to conduct physically mapping on the real place.<br>

<a href="./top.png" data-lightbox="Pcd Generation" data-title="" data-alt="Pcd Generation"><img src="./top.png"></a>

When `Pcd Generation` is conducted, `Vehicle` object (carring `LiDAR`) is warped along all centerlines of lanelets in the imported `OSM` map.
Point cloud map is generated to record points by `LiDAR` on `Vehicle` object on each centerlines of lanelets.

### Overview

### Configuration
`PcdGeneration` can be configured from `PcdGenerator` and `RGLMappingAdapter` components.

The configurable elements are listed in the following table:

PcdGenerator

| Parameter | Description |
|---|---|
| Osm Data Container | Imported `OSM` file. |
| Vehicle Transform | Game object containing sensors to capture pointcloud. |
| Rgl Mapping Adapter | Reference of `LiDAR` object which is attached `RGLMappingAdapter`. |
| Output Pcd File Path | Result `PCD` file name.<br>On captured, it will be saved in `Assets/[Output Pcd File Path]` |
| Capture Location Interval | Distance in meters between consecutive warps along the centerline of a lanelet. |
| World Origin ROS | World origin in ROS coordinate systems, will be added to every point coordinates. |

RGLMappingAdapter

| Parameter | Description |
|---|---|
| Enable Downsampling | Enable/disable point cloud data downsampling. |
| Leaf Size | Resolution of point cloud data downsampling. |

## Execution and Parameters

### Execution


### Parameters
Following parameters are useful to point cloud map of quality and file size you want to.

#### Leaf Size
Downsampling aims to reduce PCD size which for large point clouds may achieve gigabytes in exchange for map details. It is essential to find the best balance between the size and acceptable details level.

`Leaf Size` can be configured by `RGLMappingAdapter` component.

A small Leaf Size results in a more detailed PCD, while a large Leaf Size could result in excessive filtering such that objects like buildings are not recorded in the PCD.

| Leaf Size = 1.0 | Leaf Size = 10.0 | Leaf Size = 100.0 |
|---|---|---|
| ![Leaf Size 1](./leaf_size_1.png) | ![Leaf Size 10](./leaf_size_10.png) | ![Leaf Size 100](./leaf_size_100.png) \

#### Capture Location Interval
If the Capture Location Interval is too small, it could result in a sparse PCD where some region of the map is captured well but the other regions aren't captured at all.

`Capture Location Interval` can be configured by `PcdGeneration` component.

| Capture Location Interval = 6 | Capture Location Interval = 20 | Capture Location Interval = 100 |
|---|---|---|
| ![Interval 6](./interval_6.png) | ![Interval 20](./interval_20.png) | ![Interval 100](./interval_100.png) |

## Instruction
To use `Scenario Simulator Connection`, please follow the steps below.

For the preparation, the following must be prepared:

- 3D map (.fbx)
- lanelet map (.osm)
- LiDAR sensor

!!! info
    AWSIM includes `AutowareSimulationDemo` scene.<br>
    Please refer to:<br>
    * `Assets/Awsim/Scenes/PcdGenerationDemo.unity`

### 1. Add a `Vehicle` objecr
Add `Vehicle` object to carry LiDAR and capture points.<br>
In addition, visual elements of `Vehicle` if needed.

<a href="./vehicle.png" data-lightbox="Vehicle" data-title="" data-alt="Vehicle"><img src="./vehicle.png"></a>

Please create `Vehicle` object as the following:

1. Create empty `GameObject` (should be `Vehicle`)
2. (optional) Create empty `GameObject` (should be `Vehicle/Geometry`)
    1. If needed, visual elements of `Vehicle` added here
    2. Visual element can even be a simple `Cube` object as the sample image

### 2. Add a `Camera`
Add Unity `Camera` to visualize recording process.

<a href="./camera.png" data-lightbox="Camera" data-title="" data-alt="Camera"><img src="./camera.png"></a>

Please create Unity `Camera` as the following:

1. Create `Camera` object (should be `Vehicle/Follow Camera`)
2. Attach `FollowCamera` component to `Follow Camera`
3. Fill in `Target` field with `Vehicle`
4. Modify `Transform`
    5. Using `Cameras` Unity view allows you to check the camera view

### 3. Add a `LiDAR` related objects
Add `LiDAR` object and configure components to record points.

<a href="./lidar.png" data-lightbox="LiDAR" data-title="" data-alt="LiDAR"><img src="./lidar.png"></a>

#### LiDAR object

Please create and configure `LiDAR` object as the following:

1. Create `LiDAR` object (should be `Vehicle/LiDAR`)
2. Set `Transform.Position.Y` 2.5
2. Attach `PointCloudVisualization` component to `LiDAR`
3. Attach `LidarSensor` component to `LiDAR`
    1. Select `Model Preset` (`VelodyneVLP16` and `VelodyneVLS128` is recommended)
        1. `VelodyneVLS128` can create more detail maps than `VelodyneVLP16`
        2. `VelodyneVLP16` can work lighter than `VelodyneVLS128`
    2. Set `Apply Distance Gaussian Noise` and `Apply Angular Gaussian Noise` `False`
4. Attach `RGLMappingAdapter` component to `LiDAR`

#### RGLSceneManager

Please create `SceneManager` object as the following:

5. Create empty `GameObject` (should be `RGLSceneManager`)
6. Attach `SceneManager` component to `RGLSceneManager`

### 4. Setup `PcdGenerator`
Add `PcdGenerator` component to manage above objects and create point cloud map.

<a href="./pcd_generator.png" data-lightbox="PcdGenerator" data-title="" data-alt="PcdGenerator"><img src="./pcd_generator.png"></a>

1. Create empty `GameObject` (should be `PcdGenerator`)
2. Attach `PcdGenerator` component to `PcdGenerator`
3. Fill in `Osm Data Container` field with `.osm` file
    1. If there is not `.osm` file in project, move `.osm` file to `Assets/Awsim/Externals` directory using file expoler
4. Fill in `Vehicle Transform` field with `Vehicle`
5. Fill in `Rgl Mapping Adapter` field with `LiDAR`
6. (optional) Fill in `World Origin ROS` field if your map has `Mgrs Position` component
<a href="./mgrs_position.png" data-lightbox="Mgrs Position" data-title="" data-alt="Mgrs Position"><img src="./mgrs_position.png"></a>

### 7. Call methods of `PcdGenerator` and `FollowCamera`
Some methods of `PcdGenerator` and `FollowCamera` should be called from callback of `MonoBehaviour` to enable `Pcd Generation`.

Please implement as the following:

1. Create or open class which is inherit `MonoBehaviour`
2. Make fields of `PcdGenerator` and `FollowCamera`
3. Add description of calling method of `PcdGenerator` and `FollowCamera`

The method should be called are listed in the following table:

`ScenarioSimulatorClient`

| Method | Description |
|---|---|
| Initialize() | Should be called Start() callback. |
| OnUpdate() | Should be called Update() callback. |

`FollowCamera`

| Method | Description |
|---|---|
| Initialize() | Should be called Start() callback. |
| OnUpdate() | Should be called Update() callback. |

!!! info
    AWSIM includes `AutowareSimulationDemo` scene.<br>
    Please refer to:<br>
    * `Assets/Awsim/Scenes/PcdGenerationDemo/PcdGenerationDemo.cs`<br>
    * `Assets/Awsim/Scenes/PcdGenerationDemo.unity` scene

## Verify and modify `PCD` files