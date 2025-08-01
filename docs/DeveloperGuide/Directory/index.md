AWSIM has the following directory structure.

!!! info
    AWSIM has the following directory structure. Mostly they are grouped by file type. However, scene-specific assets are specifically included in the scene directory (`Assets/Awsim/Scenes/<Scene dir>`) . **These assets are not intended for use in other scenes**.


```
Awsim       //  root directory.
 │
 │
 ├─Assets                           // Unity project Assets directory.
 │  │                               // Place external libraries
 │  │                               // under this directory.
 │  │                               // (e.g. RGLUnityPlugin, ROS2ForUnity, etc..)
 │  │
 │  │
 │  ├─Awsim                         // Includes assets directly related to AWSIM
 │  |  |                            // (Scripts, Prefabs etc.)
 │  │  │
 │  │  │
 │  │  ├─Externals                  // Place for large files or
 │  │  |                            // external project dependencies
 │  │  |                            // (e.g. Ninshinjuku map asset).
 │  │  │                            // The directory is added to `.gitignore`
 │  │  │ 
 │  │  │ 
 │  │  ├─Fonts                      // Fonts including text mesh pro sdf.
 │  │  │
 │  │  │
 │  │  ├─Graphics                   // Graphics including rendering pipeline.
 │  │  │
 │  │  │
 │  │  ├─Inputactions               // Input action assets.
 │  │  │
 │  │  │
 │  │  ├─Models                     // 3D models
 │  │  │  │                         // Textures and materials for 3D models
 │  │  │  │                         // are also included.
 │  │  │  │
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
 │  │  │
 │  │  ├─Scenes                     // Scenes
 │  │  │  │                         // Includes scene-specific scripts, etc.
 │  │  │  │
 │  │  │  │
 │  │  │  └─<Scene dir>             // Directory dedicated to specific scenes.
 │  │  │                            // Assets (script, prefab, etc.) that
 │  │  │                            // can be used only in that scene are here.
 │  │  │                            // Generic assets are not included here.
 │  │  │
 │  │  │
 │  │  ├─Scripts                    // C# scripts.
 │  │  │  │
 │  │  │  │
 │  │  │  ├─Common                  // Common layer scripts.
 │  │  │  │
 │  │  │  │
 │  │  │  ├─Editor                  // Editor scripts.
 │  │  │  │
 │  │  │  │
 │  │  │  ├─Entity                  // Entity layer scripts.
 │  │  │  │
 │  │  │  │
 │  │  │  ├─UI                      // UI layer scripts.
 │  │  │  │
 │  │  │  │
 │  │  │  └─Usecase                 // Usecase layer scripts.
 │  │  │
 │  │  │
 │  │  └─Textures                   // Textures.
 │  │
 │  │
 │  ├─RGLUnityPlugin        // Robotec GPU LiDAR external library.
 │  │                       // https://github.com/RobotecAI/RobotecGPULidar
 │  │
 │  │
 │  ├─Ros2ForUnity          // ROS2 communication external library.
 │  │                       // https://github.com/RobotecAI/ros2-for-unity
 │  │
 │  └─TestMesh pro          // Text mesh pro library
 │                          // https://docs.unity3d.com/Packages/com.unity.ugui@2.0/manual/TextMeshPro/index.html
 │
 │
 ├─Packages         // Unity automatically generated directories.
 ├─ProjectSettings  //
 ├─UserSettings     //
 │
 │
 └─docs             // AWSIM documentation. Generated using mkdocs.
                    // https://www.mkdocs.org/
```