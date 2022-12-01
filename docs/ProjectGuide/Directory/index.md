# Directory

AWSIM has the following directory structure. Mostly they are grouped by file type.


```
AWSIM       //  root directory.
 │
 │
 ├─Assets                           // Unity project Assets directory.
 │  │                               // Place external libraries
 │  │                               // under this directory.
 │  │                               // (e.g. RGLUnityPlugin, ROS2ForUnity, etc..)
 │  │
 │  │
 │  ├─AWSIM                         // Includes assets directly related to AWSIM
 │  |                               // (Scripts, Prefabs etc.)
 │  │  │
 │  │  │
 │  │  ├─Externals                  // Place for large files or
 │  │  |                            // external project dependencies
 │  │  |                            // (e.g. Ninshinjuku map asset).
 │  │  │                            // The directory is added to `.gitignore`
 │  │  │
 │  │  ├─HDRPDefaultResources       // Unity HDRP default assets.
 │  │  │
 │  │  ├─Materials                  // Materials used commonly in Project.
 │  │  │
 │  │  ├─Models                     // 3D models
 │  │  │  │                         // Textures and materials for 3D models
 │  │  │  │                         // are also included.
 │  │  │  │
 │  │  │  └─<3D Model>              // Directory of each 3D model.
 │  │  │     │
 │  │  │     │
 │  │  │     ├─Materials            // Materials used in 3D model.
 │  │  │     │
 │  │  │     │
 │  │  │     └─Textures             // Textures used in 3D model.
 │  │  │
 │  │  │
 │  │  ├─Prefabs                    // Prefabs not dependent on a specific scene.
 │  │  │
 │  │  ├─Scenes                     // Scenes
 │  │  │  │                         // Includes scene-specific scripts, etc.
 │  │  │  │
 │  │  │  │
 │  │  │  ├─Main                    // Scenes used in the simulation.
 │  │  │  │
 │  │  │  │
 │  │  │  └─Samples                 // Sample Scenes showcasing components.
 │  │  │
 │  │  │
 │  │  └─Scripts                    // C# scripts.
 │  │
 │  │
 │  ├─RGLUnityPlugin        // Robotec GPU LiDAR external Library.
 │  │                       // see: https://github.com/RobotecAI/RobotecGPULidar
 │  │
 │  │
 │  └─Ros2ForUnity          // ROS2 communication external Library.
 │                          // see: https://github.com/RobotecAI/ros2-for-unity
 │
 ├─Packages         // Unity automatically generated directories.
 ├─ProjectSettings  //
 ├─UserSettings     //
 │
 │
 └─docs             // AWSIM documentation. Generated using mkdocs.
                    // see: https://www.mkdocs.org/

```