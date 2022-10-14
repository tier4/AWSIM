# Directory

AWSIM has the following directory structure. Basically, they are grouped by extension.


```
AWSIM       // AWSIM root.
 │
 │
 ├─Assets           // Unity project Assets directory.
 │  │               // Place external libraries under this directory.
 │  │               // (such as RGLUnityPlugin, ROS2ForUnity, etc..)
 │  │
 │  │
 │  ├─AWSIM                 // Includes AWSIM implementation, Asset, etc.         
 │  │  │
 │  │  │ 
 │  │  ├─Externals                  // git ignore directory.
 │  │  │                            // Place large files, 
 │  │  │                            // such as 3D models of the Environment.
 │  │  │ 
 │  │  │
 │  │  ├─HDRPDefaultResources       // Unity HDRP default assets.
 │  │  │
 │  │  │
 │  │  ├─Materials                  // Materials used commonly in Project.
 │  │  │
 │  │  │ 
 │  │  ├─Models                     // 3D models.
 │  │  │  │                         // Textures and materials for 3D models 
 │  │  │  │                         // are also included.
 │  │  │  │
 │  │  │  │
 │  │  │  └─<3D Model>                      // Directory of each 3D model.
 │  │  │     │
 │  │  │     │
 │  │  │     ├─Materials                            // Materials used in 3D model.
 │  │  │     │
 │  │  │     │
 │  │  │     └─Textures                             // Textures used in 3D model.
 │  │  │     
 │  │  │
 │  │  ├─Prefabs                    // Prefabs.
 │  │  │                            // Not dependent on a specific scene.
 │  │  │                            // Can be used in all scene.
 │  │  │
 │  │  │
 │  │  ├─Scenes                     // Scenes. 
 │  │  │  │                         // Includes scene-specific scripts, etc.
 │  │  │  │
 │  │  │  │
 │  │  │  ├─Main                            // Scenes used in the simulation.
 │  │  │  │
 │  │  │  │
 │  │  │  └─Samples                         // Sample Scenes by Component.
 │  │  │
 │  │  │
 │  │  └─Scripts                    // C# codes.
 │  │
 │  │  
 │  ├─RGLUnityPlugin        // Robotec GPU LiDAR external Library.
 │  │                       // (TODO) URL
 │  │
 │  │  
 │  └─Ros2ForUnity          // ROS2 communication external Library.
 │                          // https://github.com/RobotecAI/ros2-for-unity
 │  
 │
 ├─Packages         // Unity Auto generated directories.
 ├─ProjectSettings  //
 ├─UserSettings     //
 │
 │
 └─docs             // AWSIM document. using mkdocs. 
                    // https://www.mkdocs.org/

```