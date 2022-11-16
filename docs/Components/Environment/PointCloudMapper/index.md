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
    1. Configure mapping adapter - e.g. set leaf size for filtering.

### Setup PointCloudMapper

In your scene, create an empty GameObject and attach the `Point Cloud Mapper` component to it. To properly configure the component, please, set the following parameters from the inspector:

- `Osm Container`: the OSM file you imported in step above.
- `World Origin`: MGRS position of the origin of the scene ***Note:in ROS coordinate system, not Unity.***
- `Capture Location Interval`: Distance between consecutive capture points along lanelet centerline.
- `Output Pcd File Path`: Output relative path from Assets folder.
- `Target Vehicle`: The vehicle you want to use for point cloud capturing.

If using RGL, make sure that `RGLSceneManager` GameObject is added to the scene.

### Capture and Generate PCD

If you play simulation with a scene prepared with the steps above, `PointCloudMapper` will automatically start mapping.
The vehicle will warp along centerlines by intervals of `CaptureLocationInterval` and capture point cloud data.
PCD file will be written when you stop your scene or all locations in the route are captured.

## Sample Scene
`PointCloudMappingDemo.unity` is a sample scene for `PointCloudMapper` showcase. It requires setup of OSM data and 3D model map of the area according to the steps above.
