# PointCloudMapper

<img src=image_0.png width=700px>

## Description
PointCloudMapper is a tool for a vehicle based point cloud mapping in a simulation space.

## Required Data
- Lanelet2 format OSM data
- 3D model map of the area
- Vehicle object with sensors

## How to Use
### Import OSM
1. Drag and drop an OSM file into Unity project
2. OSM file will be imported as `OsmDataContainer`.

### Setup Vehicle Sensors [RGL]

1. Create an empty object
2. Attach script `LidarSensor.cs` to the game object
3. Configure lidar pattern, e.g. by selecting one of the available presets
4. Attach script `RGLMappingAdapter.cs` to the game object.
5. Configure mapping adapter - e.g. set leaf size for filtering.

### Setup PointCloudMapper
In your scene, create new object and add `PointCloudMapper` component. Set following parameters in the inspector.
- `Osm Container`: the OSM file you imported in the above step.
- `World Origin`: MGRS position of the origin of the scene (in ROS coordinate system, not Unity).
- `Capture Location Interval`: Distance between consecutive capture points along lanelet centerline.
- `Output Pcd File Path`: Output relative path from Assets folder.
- `Target Vehicle`: The vehicle you want to use for point cloud capturing.

If using RGL, make sure that `RGLSceneManager` game object is added to the scene (a prefab is available).

### Capture and Generate PCD
If you play your scene after the above steps, PointCloudMapper will automatically start mapping.
The vehicle will warp along centerlines by intervals of `CaptureLocationInterval` and capture point cloud data.
PCD file will be written when you stop your scene or all locations in the route are captured.

## Sample Scene
`PointCloudMappingDemo.unity` is a sample scene for `PointCloudMapper`. You need to setup OSM data and 3D model map of the area according to the above steps.
