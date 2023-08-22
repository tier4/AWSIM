# Robotec GPU Lidar Unity plugin

## Scene preparation

The native RGL library needs a once-per-scene preparation to access models on the scene from the native library:

1. Create an empty object (name it SceneManger)
2. Attach script `SceneManager.cs` to the SceneManager object

## Creating Lidar object (or prefab)

1. Create an empty object
2. Attach script `LidarSensor.cs`.
3. `PointCloudVisualization.cs` will be added automatically, however, you can disable it.
4. Now you can add a callback from another script to receive a notification when data is ready:
   ```cs
   lidarSensor = GetComponent<LidarSensor>();
   lidarSensor.OnOutputData += HandleLidarDataMethod;
   ```

## Adding new lidar models

To add a new lidar model, perform the following steps:
1. Add its name to the `LidarModels.cs`
2. If the Lidar has a non-uniform laser array construction (e.g. different linear/angular spacing between lasers), add an entry to the `LaserArrayLibrary`.
3. Add an entry to `LidarConfigurationLibrary`. Use the provided laser array or generate a uniform one using the static method `LaserArray`.Uniform()`.
4. Done. New lidar preset should be available via Unity Inspector.

## Package structure

Additionally, the package contains also:
- Plugins
  - Dynamically loaded libraries for Windows and Linux (.dll and .so)
- Resources
  - Visualization shader and material
- Scripts
  - Details below

## Scripts structure
The code consists of the following parts:
  - `LidarSensor.cs`
    - Non-ROS code, most of the high-level logic, frequency control
  - `SceneManager.cs`
    - Responsible for syncing the scene between Unity and GPU (CUDA)
  - `PointCloudVisualization.cs`
    - Displays PointCloud on the Unity Scene
  - `RGLDebugger.cs`
    - Provides configuration for Native RGL debug tools (logging and tape)
  - `SemanticCategory.cs`
    - Allows to add category ID to game objects for instance/semantic segmentation tasks
  - `IntensityTexture.cs`
    - Provides input component that can be assigned to any GameObject. It contains a slot for intensity texture.
- A set of classes providing tools to define LiDAR specification (mostly: ray poses)
  - `LidarModels.cs`
    - Enumeration of some real-world LiDARs names
  - `LidarConfiguration.cs`
    - Top-level configuration class, horizontal ranges, distance range, laser array
  - `LidarConfigurationLibrary.cs`
    - Provides a number of pre-defined LidarConfigurations
  - `LaserArray.cs`
    - Definition of a (vertical) array of lasers
  - `LaserArrayLibrary.cs`
    - Provides a number of pre-defined `LaserArrays
  - `Laser.cs`
    - Describes offsets of a single laser within a LaserArray
- LowLevelWrappers
  - Provides some convenience code to call Native RGL functions
- Utilities
  - Miscellaneous utilities to make rest of the code clearer

## Debugging Native RGL library (advanced)

1. Create an empty object
2. Attach script `RGLDebugger.cs`
3. Configure debug tools:
   - Logging - saves logs from Native RGL to the file
     - `Log Level` - logging verbosity level.
     - `Log Output Path` - path to the file where logs will be saved
   - Tape (Linux only) - saves all Native RGL functions calls to the file. For playback, it is required to have a special program (available in [RGL repository](https://github.com/RobotecAI/RobotecGPULidar)).
     - `Tape Output Path` - path to the file where tape recording will be saved (should contain filename without extension)
     - `Activate Tape Record` - tape recording activation button
4. Start the simulation

In case of any problems, please create issue in the [RGL repository](https://github.com/RobotecAI/RobotecGPULidar) and attach the generated files with logs and tape.