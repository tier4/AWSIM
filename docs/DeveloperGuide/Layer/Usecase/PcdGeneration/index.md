# PcdGeneration

## Abstract
`Pcd Generation` is a tool for a vehicle based point cloud mapping in a simulation environment.<br>
It is useful when you need a point cloud based on some location, but are not able to conduct physically mapping on the real place.<br>

<a href="./top.png" data-lightbox="Traffic Simulation" data-title="" data-alt="Traffic Simulation"><img src="./top.png"></a>

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

### 1. Setup a `Vehicle` for capturing

### 2. Setup `PointCloudMapper`

### 3. Capture and generate `PCD`

### 4. (optional) Verify the `PCD`