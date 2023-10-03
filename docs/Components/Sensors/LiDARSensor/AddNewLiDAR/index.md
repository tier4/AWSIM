# Add a new LiDAR
`RGLUnityPlugin` (`RGL`) comes with a number of the most popular *LiDARs* model definitions and [ready-to-use prefabs](../../../../Components/Sensors/LiDARSensor/LiDARSensor/#prefabs). However, there is a way to create your custom *LiDAR*. This section describes how to add a new *LiDAR* model that works with `RGL`, then create a prefab for it and add it to the scene.

!!! warning "Supported LiDARs"
    Not all lidar types are supported by `RGL`. Unfortunately, in the case of `MEMs` *LiDARs*, there is a non-repetitive phenomenon - for this reason, the current implementation is not able to reproduce their work.

## 1. Add new LiDAR model
The example shows the addition of a *LiDAR* named `NewLidarModel`.

To add a new *LiDAR* model, perform the following steps:

1. Add its name to the `LidarModels.cs` at the end of the enumeration. The order of enums must not be changed to keep existing prefabs working.

    ![lidar_models](lidar_models.png)

1. If the *LiDAR* has a non-uniform laser array construction (e.g. different linear / angular spacing between lasers), add an entry to the `LaserArrayLibrary`, otherwise, skip this step.
   
    !!! warning "Coordinate system"
        Keep in mind that *Unity* has a left-handed coordinate system, while most of the lidar's manuals use a right-handed coordinate system. In that case, reverse sign of the values of the angles.

    ![lidar_array](lidar_array.png)



1. Add an entry to `LidarConfigurationLibrary`. If the *LiDAR* has a uniform laser generate a uniform one using static method `LaserArray.Uniform()` - just like the `RangeMeter`.

    ![lidar_configuration](lidar_configuration.png)

1. Done. New *LiDAR* preset should be available via *Unity Inspector*.

    ![done](done.png)

## 2. Create new LiDAR prefab

1. Create an empty object and name it appropriately according to the *LiDAR* model.
1. Attach script `LidarSensor.cs` to created object.
1. Set the new added *LiDAR* model in `Model Preset` field, check if the configuration loads correctly. You can now customize it however you like.
1. (Optional) Attach script `PointCloudVisualization.cs` for visualization purposes.
1. For publishing point cloud via *ROS2* attach script `RglLidarPublisher.cs` script to created object.
1. Set the topics on which you want the data to be published and their frame.
2. Save the prefab in the project.

## 3. Test your prefab

1. Create a new scene (remember to add the [`SceneManager`](../../../../Components/Sensors/LiDARSensor/RGLUnityPlugin/#scenemanager)) or use one of [the existing sample scenes](../../../../ProjectGuide/Scenes/#rgl-test-scenes).
1. Add the prepared *LiDAR* prefab by drag the prefab file and drop it into a scene.

    <img src="img/AddPrefabLidar.png" width="700">

1. A *LiDAR* *GameObject* should be instantiated automatically

    <img src="img/PrefabLidarObject.png" width="700">

1. Now you can run the scene and check how your *LiDAR* works.

!!! success
    We encourage you to develop a vehicle using the new *LiDAR* you have added - learn how to do this [here](../../../../Components/Vehicle/AddNewVehicle/AddAVehicle/).
