# LidarSensor
`LidarSensor` is the component that simulates the *LiDAR* (`Light Detection and Ranging`) sensor.
*LiDAR* works by emitting laser beams that bounce off objects in the environment, and then measuring the time it takes for the reflected beams to return, allowing the sensor to create a *3D* map of the surroundings.
This data is used for object detection, localization, and mapping.

`LiDAR` in an autonomous vehicle can be used for many purposes.
The ones mounted on the top of autonomous vehicles are primarily used

- to scan the environment for localization in space
- to detect and identify obstacles such as approaching vehicles, pedestrians or other objects in the driving path.

`LiDARs` placed on the left and right sides of the vehicle are mainly used to monitor the traffic lane and detect vehicles moving in adjacent lanes, enabling safe maneuvers such as lane changing or turning.

`LidarSensor` is closely related to the external `RGL` library, which is described [here](../../../ExternalLibraries/RGLUnityPlugin/).

!!! warning "Use RGL in your scene"
    If you want to use `RGL` in your scene, make sure the scene has an [`RGLSceneManager` component](../../../ExternalLibraries/RGLUnityPlugin/#rglscenemanager) added and all objects meet the [usage requirements](../../../ExternalLibraries/RGLUnityPlugin/#usage-requirements).

!!! note "RGL default scenes"
    If you would like to see how `LidarSensor` works using `RGL` or run some tests, we encourage you to familiarize yourself with the [`RGL` test scenes section](../../../DefaultExistingScenes/#rgl-test-scenes).

!!! note "Supported *LiDARs*"
    The current scripts implementation allows you to configure the prefab for any mechanical *LiDAR*.
    You can read about how to do it [here](../../../../../DeveloperGuide/Tutorials/AddANewLiDARModel/).
    *MEMS-based LiDARs* due to their different design are not yet fully supported.

## Prefabs
Prefabs can be found under the following path:

```
Assets\AWSIM\Prefabs\RobotecGPULidars\*
```

The table of available prefabs can be found below:

| LiDAR                 | Path                     | Appearance                                       |
| :-------------------- | :----------------------- | :----------------------------------------------- |
| *HESAI Pandar40P*     | `HesaiPandar40P.prefab`  | <img src=imgs_prefabs/pandar40p.png width=150px> |
| *HESAI PandarQT64*    | `HesaiPandarQT64.prefab` | <img src=imgs_prefabs/pandarqt.png width=150px>  |
| *Ouster OS1-64*       | `OusterOS1-64.prefab`    | <img src=imgs_prefabs/os1-64.png width=150px>    |
| *Velodyne VLP-16*     | `VelodyneVLP16.prefab`   | <img src=imgs_prefabs/vlp16.png width=150px>     |
| *Velodyne VLC-32C*    | `VelodyneVLP32C.prefab`  | <img src=imgs_prefabs/vlp32.png width=150px>     |
| *Velodyne VLS-128-AP* | `VelodyneVLS128.prefab`  | <img src=imgs_prefabs/vls128.png width=150px>    |

![components](components.png)

## Link 
`LidarSensor` placed on top of the vehicle does not need to have its own frame, the data it generates is defined directly in the `sensor_kit_base_link` frame.
The sensor prefab is added to this frame.
The `sensor_kit_base_link` frame is added to the `base_link` object located in the `URDF`.

![link](link.png)

!!! warning "Additional LiDARs"
    For a *LiDAR* placed on the left side, right side or rear, an additional link should be defined.

## Scripts and Resources
The `LidarSensor` functionality is split into three scripts:

- *LidarSensor Script* - provides lidar configuration and performs native *RGL* raytrace calls,
- *RglLidarPublisher Script* - converts the data output from `LidarSensor` to *ROS2* message type and publishes it.
- *PointCloudVisualization Script* - visualizes point cloud collected by sensor.

Moreover, the scripts use `Resources` to provide configuration for prefabs of supported lidar models:

- *LaserModels* - provides a list of supported models,
- *LaserArrayLibrary* - provides data related to laser array construction for supported models,
- *LaserConfigurationLibrary* - provides full configuration, with ranges and noise for supported models.
These are elements of the `RGL` plugin, you can read more [here](../../../ExternalLibraries/RGLUnityPlugin/).

## LidarSensor Script
![script](script.png)

This is the main script in which the entire process - collecting the point clouds from the current pose and its processing by the created sequence of `RGL` nodes - takes place.

At the output of the script we get 3 categories of data.
Two of them: *rosPCL24* and *rosPCL48* are point clouds that are published by the *RglLidarPublisher Script*.
Whereas vector *onlyHits* is used for visualization by the *PointCloudVisualization Script*.

*rosPCL24* is a 24-byte point cloud format used by *Autoware*.
While *rosPCL48* is its 48-byte extended version.
Details on the construction of these formats are available in the `PointCloudFormats` under the following path:

```
AWSIM/Assets/AWSIM/Scripts/Sensors/LiDAR/PointCloudFormats.cs
```

!!! note "*rosPCL48* format"
    For a better understanding of the *rosPCL48* format, we encourage you to familiarize yourself with the point cloud pre-processing process in *Autoware*, which is described [here](https://autowarefoundation.github.io/autoware-documentation/latest/design/autoware-architecture/sensing/data-types/point-cloud/#channel).

### Output Data
|  Category  |    Type    | Description                                                                               |
| :--------: | :--------: | :---------------------------------------------------------------------------------------- |
| *onlyHits* | Vector3[ ] | Vertices for visualization in *Unity's* coordinate system                                 |
| *rosPCL24* |  byte[ ]   | Vertices for publishing *Autoware* format pointcloud in *ROS2* coordinate system          |
| *rosPCL48* |  byte[ ]   | Vertices for publishing extended *Autoware* format pointcloud in *ROS2* coordinate system |

### Elements configurable from the editor level
- `Automatic Capture Hz` - the rate of sensor processing (default: `10Hz`)
- `Model Preset` - allows selecting one of the built-in *LiDAR* models (default: `RangeMeter`)
- `Apply Distance Gaussian Noise` - enable/disable distance gaussian noise (default: `true`)
- `Apply Angular Gaussian Noise` - enable/disable angular gaussian noise (default: `true`)
- *Configuration*:
    - `Laser Array` - geometry description of lidar array, should be prepared on the basis of the manual for a given model of *LiDAR* (default: loaded from `LaserArrayLibrary`)
    - `Horizontal Steps` - the number of laser array firings between `Min H Angle` and `Max H Angle` (default: `1`)
    - `Min H Angle` - minimum horizontal angle, left (default: `0`)
    - `Max H Angle` - maximum horizontal angle, right (default: `0`)
    - `Max Range` - maximum range of the sensor (default: `40`)
    - *Noise Params*: 
        - `Angular Noise Type` - angular noise type<br>(default: `Ray Based`)
        - `Angular Noise St Dev` - angular noise standard deviation in degree<br>(default: `0.05729578`)
        - `Angular Noise Mean` - angular noise mean in degrees<br>(default: `0`)
        - `Distance Noise St Dev Base` - distance noise standard deviation base in meters<br>(default: `0.02`)
        - `Distance Noise Rise Per Meter` - distance noise standard deviation rise per meter<br>(default: `0`)
        - `Distance Noise Mean` - distance noise mean in meters<br>(default: `0`)

## RglLidarPublisher Script
![script_ros2](script_ros2.png)

This is a script developed to directly integrate `R2FU` with `RGL`.
It ensures the creation of a separate *ROS2* node named `/RobotecGPULidar` with publishers for point clouds generated by `RGL`.

Thanks to this integration, such a large amount of data can be published with sufficient frequency without causing large performance overheads.
!!! note "R2FU"
    We encourage you to read more about `R2FU` in this [section](../../../ExternalLibraries/Ros2Unity/).

### Published Topics
- Frequency: `10Hz`
- QoS:  `Reliable`, `Volatile`, `Keep last/1`

|         Category          | Topic                  | Message type               | `frame_id` |
| :-----------------------: | :--------------------- | :------------------------- | :--------: |
| PointCloud 24-byte format | `/lidar/pointcloud`    | `sensor_msgs/PointCloud2`  |  `world`   |
| PointCloud 48-byte format | `/lidar/pointcloud_ex` | `sensor_msgs/PointCloud2 ` |  `world`   |

### Elements configurable from the editor level
- `Pcl 24 Topic` - the *ROS2* topic on which the [`PointCloud2`](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud.html) message is published<br>(default: `"/sensing/lidar/top/pointcloud_raw"`)
- `Pcl 48 Topic` - the *ROS2* topic on which the [`PointCloud2`](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud.html) message is published<br>(default: `"/sensing/lidar/top/pointcloud_raw_ex"`)
- `Frame ID` - frame in which data are published, used in [`Header`](https://docs.ros2.org/latest/api/std_msgs/msg/Header.html) (default: `"world"`)
- `Publish PCL24` - if publish cloud *PCL24* (default: `true`)
- `Publish PCL48` - if publish cloud *PCL48* (default: `true`)
- `Qos Settings` - Quality of service profile used in the publication<br>(default: `Best effort`, `Volatile`, `Keep last`, `5`)

## PointCloudVisualization Script
![script_visualization](script_visualization.png)

A script visualizing a point cloud - obtained from `RGL` in the form of a [`Vector3`](https://docs.unity3d.com/ScriptReference/Vector3.html) list - as colored points in the *Unity* scene.
Based on the defined color table, it colors the points depending on the height at which they are located.

The obtained points are displayed as the vertices of mesh, and their coloring is possible thanks to the use of `PointCloudMaterial` material which can be found in the following path:
```
AWSIM/Assets/RGLUnityPlugin/Resources/PointCloudMaterial.mat
```

`Point Cloud Visualization` preview:

<img src="LidarVisualizationOnScene.png" width="700">

### Elements configurable from the editor level
- `Point Shape` - the shape of the displayed points (default: `Box`)
- `Point Size` - the size of the displayed points (default: `0.05`)
- `Colors` - color list used depending on height<br>(default: `6` colors: `red, orange, yellow, green, blue, violet`)
- `Auto Compute Coloring Heights` - automatic calculation of heights limits for a list of colors (default: `false`)
- `Min Coloring Height` - minimum height value from which color matching is performed, below this value all points have the first color from the list (default: `0`)
- `Max Coloring Height` - maximum height value from which color matching is performed, above this value all points have the last color from the list (default: `20`)
