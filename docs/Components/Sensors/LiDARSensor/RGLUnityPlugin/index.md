# RGLUnityPlugin
[*Robotec GPU Lidar*](https://github.com/RobotecAI/RobotecGPULidar) (`RGL`) is an open source high performance lidar simulator running on *CUDA*-enabled *GPUs*.
It is a cross-platform solution compatible with both *Windows* and *Linux* operating systems.
`RGL` utilizes `RTX` cores for acceleration, whenever they are accessible.

`RGL` is used in *AWSIM* for performance reasons.
Thanks to it, it is possible to perform a large number of calculations using the *GPU*, which is extremely helpful due to the size of the scenes.
*AWSIM* is integrated with `RGL` out-of-the-box - using `RGLUnityPlugin` asset.

!!! warning
    If you want to use `RGL` in your scene, make sure the scene has an [`RGLSceneManager` component](#scenemanager) added and all objects meet the [usage requirements](#usage-requirements).

## Concept
Describing the concept of using `RGL` in *AWSIM*, we distinguish:

- *Mesh* - a handle to the *on-GPU* data of the *3D* model of objects that in *AWSIM* are provided in the form of [Mesh Filter](https://docs.unity3d.com/Manual/class-MeshFilter.html) component.
`RGLUnityPlugin` supports two types of meshes: static (rendered by [Mesh Renderer](https://docs.unity3d.com/ScriptReference/MeshRenderer.html)) and animated (rendered by [Skinned Mesh Renderer](https://docs.unity3d.com/ScriptReference/SkinnedMeshRenderer.html)).
Static meshes could be shared between Entities.

- *Entity* - represents a *3D* object on the scene with its position and rotation.
It consists of a lightweight reference to a *Mesh* and a transformation matrix of the object.

- *Scene* - a location where raytracing occurs.
It is a set of entites uploaded by `SceneManager` script to the RGL Native Library.

- *Node* - performs specific operations such as setting rays for raytracing, transforming rays, performing raytracing, and manipulating output formats.
In *AWSIM*, the main sequence of `RGL` nodes that simulates *LiDAR* is created in the `LidarSensor` script.
Other scripts usually create nodes to get requested output or preprocess point cloud, and then connect those nodes to the `LidarSensor`.

- *Graph* - a collection of connected *Nodes* that can be run to calculate results.
It allows users to customize functionality and output format by adding or removing *Nodes*.

Producing a point cloud is based on the use of a *Scene* containing *Entities* with *Meshes*, and placing an *Ego* *Entity* with *LiDAR* sensor that creates a *Graph* describing ray pattern and performing raytracing.
In subsequent frames of the simulation, `SceneManager` synchronizes the scene between *Unity* and `RGL`, and *LiDAR* sensor updates rays pose on the scene and triggers *Graph* to perform raytracing and format desired output.

## Package structure
`RGLUnityPlugin` asset contains:

- *Plugins* - dynamically loaded libraries for *Windows* and *Linux* (`*.dll` and `*.so` files).
- *Resources* - visualization shader and material.
- *Scripts* - scripts for using `RGL` in the *Unity* - details below.

### Scripts
- `SceneManager` - responsible for syncing the scene between *Unity* and `RGL`.
- `LidarSensor` - provide lidar configuration and create `RGL` pipeline to simulate lidar.
- `PointCloudVisualization` - visualize point cloud on the *Unity* scene.
- `IntensityTexture` - adds slot for `Intensity Texture ID` to the *GameObject*
- `SemanticCategory` - adds category ID to the *GameObject*
- `RGLDebugger` - provides configuration for Native `RGL` debug tools (logging and tape).
- A set of classes providing tools to define *LiDAR* specification (mostly: ray poses):
    - `LidarModels` - enumeration of some real-world *LiDARs* names.
    - `LidarConfiguration` - top-level configuration class, horizontal ranges, distance range, laser array.
    - `LidarConfigurationLibrary` - provides a number of pre-defined `LidarConfigurations`.
    - `LaserArray` - definition of a (vertical) array of lasers.
    - `LaserArrayLibrary` - provides a number of pre-defined `LaserArrays`.
    - `Laser` - describes offsets of a single laser within a `LaserArray`.
    - `LidarNoiseParams` - describes a LiDAR noise that can be simulated
- `LowLevelWrappers` scripts - provides some convenience code to call Native `RGL` functions.
- `Utilities` scripts - miscellaneous utilities to make rest of the code clearer.

## SceneManager
Each scene needs `SceneManager` component to synchronize models between *Unity* and `RGL`.
On every frame, it detects changes in the *Unity's* scene and propagates the changes to native `RGL` code.
When necessary, it obtains *3D* models from *GameObjects* on the scene, and when they are no longer needed, it removes them.

Three different strategies to interact with in-simulation *3D* models are implemented.
`SceneManager` uses one of the following policies to construct the scene in `RGL`:

- `Only Colliders` - data is computed based on the colliders only, which are geometrical primitives or simplified *Meshes*.
This is the fastest option, but will produce less accurate results, especially for the animated entities.
- `Regular Meshes And Colliders Instead Of Skinned` - data is computed based on the regular meshes for static *Entities* (with [`MeshRenderers`](https://docs.unity3d.com/Manual/class-MeshRenderer.html) component) and the colliders for animated *Entities* (with [`SkinnedMeshRenderer`](https://docs.unity3d.com/Manual/class-SkinnedMeshRenderer.html) component).
This improves accuracy for static *Entities* with a negligible additional performance cost.
- `Regular Meshes And Skinned Meshes` - uses regular meshes for both static and animated *Entities*.
This incurs additional performance, but produces the most realistic results.

| Mesh Source Strategy                              | Static *Entity* | Animated *Entity* (NPC) |
| ------------------------------------------------- | --------------- | ----------------------- |
| `Only Colliders`                                  | Collider        | Collider                |
| `Regular Meshes And Colliders Instead Of Skinned` | Regular Mesh    | Collider                |
| `Regular Meshes And Skinned Meshes`               | Regular Mesh    | Regular Mesh            |

Mesh source can be changed in the `SceneManager` script properties:

<img src="scene_manager.png" width="55%">

!!! warning "Performance"
    `SceneManager` performance depends on mesh source option selected.
    
### Usage requirements
Objects, to be detectable by `RGL`, must fulfill the following requirements:

1. Contain one of the components: [`Collider`](https://docs.unity3d.com/ScriptReference/Collider.html), [`Mesh Renderer`](https://docs.unity3d.com/Manual/class-MeshRenderer.html), or [`Skinned Mesh Renderer`](https://docs.unity3d.com/Manual/class-SkinnedMeshRenderer.html) - it depends on `SceneManager` mesh source parameter.
2. Be readable from *CPU*-accessible memory - it can be achieved using the `Read/Write Enabled` checkbox in mesh settings.

    !!! note "Readable objects"
        [Primitive Objects](https://docs.unity3d.com/Manual/PrimitiveObjects.html) are readable by default.

    !!! example
        The activated *Readable* option in the mesh should look like this.

        <img src="readable.png" width="75%">

<!-- ### Debugging Native RGL library (*advanced*)

1. Create an empty object
2. Attach script `RGLDebugger`
3. Configure debug tools:
   - Logging - saves logs from Native RGL to the file
     - `Log Level` - logging verbosity level.
     - `Log Output Path` - path to the file where logs will be saved
   - Tape (Linux only) - saves all Native RGL functions calls to the file. For playback, it is required to have a special program (available in [RGL repository](https://github.com/RobotecAI/RobotecGPULidar)).
     - `Tape Output Path` - path to the file where tape recording will be saved (should contain filename without extension)
     - `Activate Tape Record` - tape recording activation button
4. Start the simulation

In case of any problems, please create issue in the [RGL repository](https://github.com/RobotecAI/RobotecGPULidar) and attach the generated files with logs and tape. -->
