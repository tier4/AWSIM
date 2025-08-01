`CameraSensor` is a component that simulates an *RGB* camera.
Autonomous vehicles can be equipped with many cameras used for various purposes.
In the current version of *AWSIM*, the camera is used primarily to provide the image to the traffic light recognition module in *Autoware*.

## Prefab

```{.yml .no-copy}
Assets/Awsim/Entity/Sensor/Camera/CameraSensor.prefab
```
```{.yml .no-copy}
Assets/Awsim/Entity/Sensor/Camera/CameraSensorScheduler.prefab
```

<br>

## CameraSensorScheduler class

The `CameraSensorScheduler` class schedules the rendering of `CameraSensor`. Normally, camera rendering is a highly demanding process. Therefore, the use of multiple camera sensors is performed by the CameraSensorScheduler by staggering the rendering timings one frame at a time.

### Setting parameters

|Type|Parameter|Feature|
|:--|:--|:--|
|`ICameraSensor`|`_schedulableCameraSensorComponents`|`CameraSensor` instances. |
|`int`|`_outputHz`|Period to output.|

<br>

## CameraSensor class

The `CameraSensor` class outputs a rendered image with OpenCV parameters applied; it is used by Autoware to recognize traffic lights, for example.

### How to use

1. Place `CameraSensorScheduler.prefab` in the scene. (It will usually be placed under the game object of the vehicle.)
1. Place `CameraSensor.prefab` in the lower level of `CameraSensorScheduler.prefab`. Then reference it in the `CameraSensorScheduler` inspector.
1. After the scene is executed, `CameraSensorScheduler` is initialized by one of the classes.
1. The `CameraSensorScheduler` coroutine then continues scheduling `CameraSensor` rendering and output.

### Setting parameters

|Type|Parameter|Feature|
|:--|:--|:--|
|`int`|`_width`|Image width (px).|
|`int`|`_height`|Image height (px).|
|`float`|`_k1`|Distortion coefficient. For plumb_bob models's k1 parameter.|
|`float`|`_k2`|Distortion coefficient. For plumb_bob models's k2 parameter.|
|`float`|`_p1`|Distortion coefficient. For plumb_bob models's p1 parameter.|
|`float`|`_p2`|Distortion coefficient. For plumb_bob models's p2 parameter.|
|`float`|`_p3`|Distortion coefficient. For plumb_bob models's k3 parameter.|
|`Camera`|`_camera`|Unity's camera component.|
|`bool`|`_enableLensDistortionCorrection`|Flag whether to apply OpenCV distortion.|
|`float`|`_sharpeningStrength`|Sharpness.|


### Output data

It is contained in the CameraSensor.IReadOnlyOutputData type.

|Type|Parameter|Feature|
|:--|:--|:--|
|`IReadOnlyCameraParameters`|`CameraParameters`||
|`RenderTexture`|`OutputRenderTexture`||

### Add output callback

`CameraSensor` can add an `Action` type callback that takes `CameraSensor.IReadOnlyOutputData` as an argument.

```cs
// sample code from CameraRos2Publisher.cs
_cameraSensor.OnOutput += Publish;
```


<br>

## CameraRos2Publisher class

`CameraRos2Publisher` converts the output of `CameraSensor` to ROS2 and publishes the topic. Publish two msgs: camera info and image.

### Setting parameters

|Type|Parameter|Feature|
|:--|:--|:--|
|`string`|`_imageTopic`|`sensor_msgs/Image` msg topic name. |
|`string`|`_cameraInfoTopic`|`sensor_msgs/CameraInfo` msg topic name.|
|`string`|`_frameId`|Frame ID of ros2.|
|`QosSettings`|`_qosSettings`|Quality of Service settings of ros2.|
|`CameraSensor`|`_cameraSensor`|Target `CameraSensor` instance.|



### Default publish topics

`CameraRos2Publisher` is configured by default with the following two topics publish. 

| Topic                                       | Message type                                                                                                                                                     | `frame_id`                              | `Hz`  | `QoS`                                                               |
|--:------------------------------------------|--:---------------------------------------------------------------------------------------------------------------------------------------------------------------|--:--------------------------------------|--:-:--|--:------------------------------------------------------------------|
| `/sensing/camera/traffic_light/camera_info` | [`sensor_msgs/CameraInfo`](https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html)                                                                     | `traffic_light_left_camera/camera_link` | `10`  | <ul><li>`Best effort`</li><li>`Volatile`</li><li>`Keep last/1`</li> |
| `/sensing/camera/traffic_light/image_raw`   | [`sensor_msgs/Image`](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)                                                                               | `traffic_light_left_camera/camera_link` | `10`  | <ul><li>`Best effort`</li><li>`Volatile`</li><li>`Keep last/1`</li> |