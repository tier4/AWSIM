# Create a PCD from Mesh

!!! warning "This section"
    This section is still under development!
<!-- !!! Draft-note
    - Reasons for making pcd from mesh instead of using real world pcd.
    - Usage requirements (osm file, 3d model (prefab))
    - Create a scene with 3D model map of the area (**gif**)
    - Import osm file (**gif**)
    - Add a simple vehicle
        - Add a geometry with collider (**screen**)
        - Add a lidar link (**screen**)
        - Add a Lidar Sensor Script (**screen**)
        - Add a RGL Mapping Adapter Script (explanation, leaf size)
        - Add a Point Cloud Visualization Script
    - Add a Scene Manager (hyperlink)
    - Add a Point Cloud Mapper Script (description, osm container, world origin, vehicle, output, interval)
    - Mapping (description play->stop->result, example - **video**)

    Remember to add pcd downsampling:<br>
    `pcl_voxel_grid output_mgrs_local.pcd output_leaf_0_2_mgrs_local.pcd -leaf 0.2 0.2 0.2`
    `pcl_convert_pcd_ascii_binary output_leaf_0_2_mgrs_local.pcd output_ascii_mgrs_local.pcd 0` -->


# PointCloudMapper

<img src=image_0.png width=700px>

## Description

PointCloudMapper is a tool for a vehicle based point cloud mapping in a simulation environment.

## Required Data

To properly perform the mapping, make sure you have the following files downloaded and configured:

- Lanelet2 format OSM data
- 3D model map of the area
- Configured in-simulation vehicle object with sensors attached

## Tool Usage

The following sections describe steps needed to properly use the tooling.

### Import OSM
1. Drag and drop an OSM file into Unity project
2. OSM file will be imported as `OsmDataContainer`.

### Setup Vehicle Sensors [RGL]

1. Create an empty GameObject
2. Attach `LidarSensor.cs` script to previously created empty GameObject
    1. Configure lidar pattern, e.g. by selecting one of the available presets
4. Attach script `RGLMappingAdapter.cs` to previously created empty GameObject
    1. Configure mapping adapter - e.g. set `Leaf Size` for filtering.

**Effect of `Leaf Size` to Point Cloud Data (PCD) generation**

A small `Leaf Size` could result in a noisy PCD, while a large `Leaf Size` could result in excessive filtering such that objects like buildings are not recorded in the PCD.

In the following examples, it can be observed that when a `Leaf Size` is 1.0, point clouds exist on roads in which they shouldn't appear.
When a `Leaf Size` is 100.0, buildings are filtered out and results in an empty PCD.
A `Leaf Size` of 10.0 results in a reasonable PCD in the given example.


| Leaf Size = 1.0                   | Leaf Size = 10.0                  | Leaf Size = 100.0                 |
| --------------------------------- | --------------------------------- | --------------------------------- |
| <img src=image_1.png width=250px> | <img src=image_2.png width=250px> | <img src=image_3.png width=250px> |


### Setup PointCloudMapper

In your scene, create an empty GameObject and attach the `Point Cloud Mapper` component to it. To properly configure the component, please, set the following parameters from the inspector:

- `Osm Container`: the OSM file you imported in step above.
- `World Origin`: MGRS position of the origin of the scene ***Note:in ROS coordinate system, not Unity.***
- `Capture Location Interval`: Distance between consecutive capture points along lanelet centerline.
- `Output Pcd File Path`: Output relative path from Assets folder.
- `Target Vehicle`: The vehicle you want to use for point cloud capturing.

If using RGL, make sure that `RGLSceneManager` GameObject is added to the scene.

**Effect of `Capture Location Interval` to PCD generation**

If the `Capture Location Interval` is too small, it could result in a sparse PCD where some region of the map is captured well but the other regions aren't captured at all.

In the below example, `Leaf Size` of 0.2 was used. Please note that using a different combination of `leaf size` and `Capture Location Interval` may result in a different PCD.


| Capture Location Interval = 6     | Capture Location Interval = 20    | Capture Location Interval = 100   |
| --------------------------------- | --------------------------------- | --------------------------------- |
| <img src=image_4.png width=250px> | <img src=image_5.png width=250px> | <img src=image_6.png width=250px> |

### Capture and Generate PCD

If you play simulation with a scene prepared with the steps above, `PointCloudMapper` will automatically start mapping.
The vehicle will warp along centerlines by intervals of `CaptureLocationInterval` and capture point cloud data.
PCD file will be written when you stop your scene or all locations in the route are captured.

## Sample Scene
`PointCloudMapping.unity` is a sample scene for `PointCloudMapper` showcase. It requires setup of OSM data and 3D model map of the area according to the steps above.

