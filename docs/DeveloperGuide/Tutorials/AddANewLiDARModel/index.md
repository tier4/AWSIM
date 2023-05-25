<!-- TODO everything -->
## Scene preparation

The native RGL library needs a once-per-scene preparation to access models on the scene from the native library:

1. Create an empty object (name it SceneManger)
2. Attach script `SceneManager.cs` to the SceneManager object


## Creating Lidar object (or prefab)

1. Create an empty object
2. Attach script `LidarSensor.cs`.
3. `PointCloudVisualization.cs` will be added automatically, however, you can disable it.
4. Now you can add a callback from another script to receive notification when data is ready:
   ```cs
   lidarSensor = GetComponent<LidarSensor>();
   lidarSensor.OnOutputData += HandleLidarDataMethod;
   ```


## Adding new lidar models

To add a new lidar model, perform the following steps:
1. Add its name to the `LidarModels.cs`
2. If the Lidar has a non-uniform laser array construction (e.g. different linear / angular spacing between lasers), add an entry to the `LaserArrayLibrary`.
3. Add an entry to `LidarConfigurationLibrary`. Use the provided laser array or generate a uniform one using static method `LaserArray.Uniform()`.
4. Done. New lidar preset should be available via Unity Inspector.
!!! Draft-note    
    - Update LidarModels.cs (description, **screen**)
    - Update LaserArrayLibrary.cs (description, using LaserArray.Uniform() to generate a array, **screens**)
    - Update LidarConfigurationLibrary.cs (description, adding the generated array, **screen**)
    - Create a prefab (**screen**)
        - Add a Lidar Sensor Script
        - Add a Rgl Lidar Publisher Script
        - Add a Point Cloud Visualization Script
    - How to test - **video** (hyperlink to 5.2)

    Could you describe what kinds of LiDARs can be supported ? 
    I think as of now mechanical LiDARs are supported but MEMs LiDARs are not yet supported. 
