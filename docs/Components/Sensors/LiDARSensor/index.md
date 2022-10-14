# Lidar Sensor
AWSIM uses Robotec GPU Lidar, which is a cross-platform (Windows and Linux), RTX-accelerated, CUDA/C++ library developed by [Robotec.AI](https://robotec.ai/). For more info on RGL, visit [its repository](https://github.com/RobotecAI/RobotecGPULidar).

To use RGL in AWSIM, Unity integration is prepared (RGLUnityPlugin).

## Output Data
`LidarSensor.OutputData` properties

|field|type|feature|
|:--|:--|:--|
|hitCount|int|Number of rays that actually has hit anything|
|hits|Vector3 [ ]|Vertices for visualization (Unity coordinate frame)|
|rosPCL24|byte [ ]|Vertices for publishing Autoware format pointcloud, ROS coordinate frame|
|rosPCL48|byte [ ]|Vertices for publishing extended Autoware format pointcloud, ROS coordinate frame|

## ROS2 Publish Topics
Topics published by `RglLidarPublisher`

|topic|msg|frame_id|hz|QoS|
|:--|:--|:--|:--|:--|
|`/lidar/pointcloud`|`sensor_msgs/PointCloud2`|`world`|`10`|`Reliable`, `Volatile`, `Keep last/1`|
|`/lidar/pointcloud_ex`|`sensor_msgs/PointCloud2 `|`world`|`10`|`Reliable`, `Volatile`, `Keep last/1`|

## Minimal scene

The scene `Assets/AWSIM/Scenes/Samples/LidarSceneDevelop.unity` can be used as a complete, minimalistic example of how to setup RGL. It contains RGLSceneManager, four lidars, and an environment composed of floor and walls.

<img src="img/LidarSceneDevelop.png" width="700">

## RGLSceneManager
Each scene needs RGLSceneManager for synchronizing models between Unity and RGL. On every frame, it detects changes happening on the scene and reacts appropriately. Three different strategies to obtain 3D model for game objects are implemented. RGLSceneManager yields a collection of RGL objects found in provided game objects, based on:

- `Only Colliders` - active colliders only,
- `Regular Meshes And Colliders Instead Of Skinned` - mesh for non-skinned MeshRenderers or set of colliders (if provided) attached to the rootBone and below for SkinnedMeshRenderers,
- `RegularMeshesAndSkinnedMeshes` - mesh for both MeshRenderer and SkinnedMeshRenderer.

Mesh source can be changed in the Scene Manager (Script) properties:

<img src="img/RGLSceneManagerModes.png" width="300">

*Note: RGLSceneManager performance depends on mesh source option selected.*

Setup instructions for new scenes:

1. Create an empty object (name it RGLSceneManager).
2. Attach script `SceneManager.cs` to the RGLSceneManager object.

## Lidar
Lidar objects typically consist of three scripts:

- `LidarSensor` - provides lidar configuration and performs raytrace
- `PointCloudVisualization` - visualizes point cloud collected by lidar
- `RglLidarPublisher` - converts the data output from LidarSensor to ROS2 msg and publishes it

To use one of the prepared prefab lidars, drag the prefab file and drop it into a scene:

<img src="img/AddPrefabLidar.png" width="700">

After that the new object with scripts, colliders, and mesh will be created:

<img src="img/PrefabLidarObject.png" width="700">

Next, you can modify scripts parameters in Unity Inspector:

In `LidarSensor` lidar configuration can be changed:

- `Automatic Capture Hz` - the rate of sensor processing
- `Model Preset` - allows selecting one of the built-in LiDAR models
- `Apply Gaussian Noise` - enable/disable gaussian noise
- `Configuration` - advanced lidar configuration (in most cases no need to change)
    - `Laser Array` - geometry description of lidar array
    - `Horizontal Steps` - the number of laser array firings between `Min H Angle` and `Max H Angle`
    - `Min H Angle` - minimum horizontal angle (left)
    - `Max H Angle` - maximum horizontal angle (right)
    - `Max Range` - maximum range of the sensor
    - `Noise Params` - lidar noise paramteres

<img src="img/LidarSensorProp.png" width="500">

In the script `Point Cloud Visualization` the material of points can be changed. If material is `None` then `PointCloudMaterial` from `Assets/RGLUnityPlugin/Resources` will be loaded. You can disable visualization by deactivating the script.

<img src="img/VisualizationProp.png" width="500">

Point Cloud Visualization preview:

<img src="img/LidarVisualizationOnScene.png" width="700">

In the last script - `RglLidarPublisher` - ROS properties such as topics names, frame ID, publish activation or QoS settings can be modified:

<img src="img/PublisherProp.png" width="500">

#### Adding new lidar models

To add a new lidar model, perform the following steps:

1. Add its name to the `LidarModels.cs`
2. If the Lidar has a non-uniform laser array construction (e.g. different linear/angular spacing between lasers), add an entry to the `LaserArrayLibrary`.
3. Add an entry to `LidarConfigurationLibrary`. Use the provided laser array or generate a uniform one using static method `LaserArray.Uniform()`.
4. Done. New lidar preset should be available via Unity Inspector.

#### Creating Lidar object (or prefab)

1. Create an empty object
2. Attach script `LidarSensor.cs`.
3. `PointCloudVisualization.cs` will be added automatically, however, you can disable it.
4. Now you can add a callback from another script to receive a notification when data is ready:
   ```cs
   lidarSensor = GetComponent<LidarSensor>();
   lidarSensor.OnOutputData += HandleLidarDataMethod;
   ```
5. For publishing point cloud via ROS2 attach script `RglLidarPublisher.cs`

#### Prefabs
Four prefabs were created with different lidar model configurations. Paths:<br>
`Assets/AWSIM/Prefabs/Sensors/RobotecGPULidars/HesaiPandar40P.prefab`<br>
`Assets/AWSIM/Prefabs/Sensors/RobotecGPULidars/HesaiPandarQT64.prefab`<br>
`Assets/AWSIM/Prefabs/Sensors/RobotecGPULidars/VelodyneVLP16.prefab`<br>
`Assets/AWSIM/Prefabs/Sensors/RobotecGPULidars/VelodyneVLS128.prefab`

## Objects requirements
Objects, to be detectable by Robotec GPU lidar, must fulfill the following requirements:

1. Contain one of the components: Collider, Mesh Renderer, or Skinned Mesh Renderer. It depends on RGLSceneManager mesh source parameter.
2. Be readable from CPU-accessible memory. It can be achieved using the “Read/Write Enabled” checkbox in mesh settings. *Note: Primitive Objects are readable by default.*

<img src="img/ReadableMeshSetting.png" width="400">

## Scripts
Paths:<br>
`Assets/RGLUnityPlugin/Scripts/SceneManager.cs`<br>
`Assets/RGLUnityPlugin/Scripts/LidarSensor.cs`<br>
`Assets/RGLUnityPlugin/Scripts/PointCloudVisualization.cs`<br>
`Assets/AWSIM/Scripts/Sensors/LiDAR/RglLidarPublisher.cs`

|script|feature|
|:--|:--|
|SceneManager.cs|Synchronize the scene between Unity and RGL.|
|LidarSensor.cs|Lidar Sensor. Provide lidar configuration and collect point cloud.|
|PointCloudVisualization.cs|Visualize point cloud collected by lidar.|
|RglLidarPublisher.cs|Convert the data output from LidarSensor to ROS2 msg and publish.|
