# CameraSensor
`CameraSensor` is a component that simulates an *RGB* camera. Autonomous vehicles can be equipped with many cameras used for various purposes. In the current version of *AWSIM*, the camera is used primarily to provide the image to the traffic light recognition module in *Autoware*.

#### Prefab
Prefab can be found under the following path:<br> `Assets\AWSIM\Prefabs\Sensors\CameraSensor.prefab`

![components](components.png)

#### Link
The mentioned single `CameraSensor` has its own frame `traffic_light_left_camera/camera_link` in which its data is published, the sensor prefab is added to it. 
All is added to the `Ego` prefab, to the `base_link` object located in the `URDF`.

![link](link.png)


#### Scripts

For the `CameraSensor` to work properly, the *GameObject* to which the scripts are added must also have:

- [*Camera component*](https://docs.unity3d.com/Manual/class-Camera.html) - the basic component that ensures the functionality of the camera as a device in *Unity* that capture and display the world to the player.
- [*HDAdditionalCameraData Script*](https://docs.unity3d.com/Packages/com.unity.render-pipelines.high-definition@13.1/api/UnityEngine.Rendering.HighDefinition.HDAdditionalCameraData.html) - additional component that holds [*HDRP*](https://docs.unity3d.com/Packages/com.unity.render-pipelines.high-definition@16.0/manual/index.html) specific parameters for camera. This Component should be added automatically together with [*Camera component*](https://docs.unity3d.com/Manual/class-Camera.html).

!!! tip "TrafficLights recognition"
    In case of problems with the recognition of traffic lights in *Autoware*, it may help to increase the image resolution and focal length of the camera in *AWSIM*.

!!! tip "Camera settings"
    If you would like to adjust the image captured by the camera, we encourage you to read [this manual](https://docs.unity3d.com/Packages/com.unity.render-pipelines.high-definition@11.0/manual/HDRP-Camera.html).


The `CameraSensor` functionality is split into two scripts:

- *CameraSensor Script* - acquires the image from the *Unity* [camera](https://docs.unity3d.com/ScriptReference/Camera.html), transforms it and saves to the  *BGR8* format, this format along with the camera parameters is its script output - script also calls the callback for it.
- *CameraSensor Script* - provides the ability to publish `CameraSensor` output as [Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html) and [CameraInfo](https://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html) messages type published on a specific *ROS2* topics.


Scripts can be found under the following path:<br>`Assets\AWSIM\Scripts\Sensors\CameraSensor\*`<br>
In the same location there are also `*.compute` files containing used [`ComputeShaders`](https://docs.unity3d.com/ScriptReference/ComputeShader.html).

## CameraSensor Script
![script](script.png)

Core camera sensor component. It is responsible for applying *OpenCV* distortion and encoding to *BGR8* format. The distortion model is assumed to be *Plumb Bob*. The script renders the image from the [camera](https://docs.unity3d.com/ScriptReference/Camera.html) to [`Texture2D`](https://docs.unity3d.com/ScriptReference/Texture2D.html), transforms it using the distortion parameters, this image is displayed in the *GUI* and further processed to obtain the list of bytes in *BGR8* format on the script output.

The script uses two [`ComputeShaders`](https://docs.unity3d.com/ScriptReference/ComputeShader.html), they are located in the same location as the scripts:

- `CameraDistortion` - to correct the image using the camera distortion parameters,
- `RosImageShader` - to encode two pixels color (*bgr8* - 3 bytes) into one (*uint32* - 4 bytes) in order to produce *ROS Image* *BGR8* buffer. 

    ![compute](compute.png)

#### Elements configurable from the editor level

- `Output Hz` - frequency of output calculation and callback (default: `10Hz`)
- *Image On GUI*:
    - `Show` - if camera image should be show on *GUI* (default: `true`)
    - `Scale` - scale of reducing the image from the camera, `1` - will give an image of real size, `2` - twice smaller, etc. (default: `4`)
    - `X Axis` - position of the upper left corner of the displayed image in the X axis, `0 `is the left edge (default: `0`)
    - `Y Axis` - position of the upper left corner of the  displayed image in the Y axis, `0` is the upper edge (default: `0`)
- *Camera parameters*
    - `Width` - image width (default: `1920`)
    - `Height` - image height (default: `1080`)
    - `K1, K2, P1, P2, K3` - camera distortion coefficients for *Plum Bob* model<br>(default: `0, 0, 0, 0, 0`)
- `Camera Object` -  reference to the basic [*Camera component*](https://docs.unity3d.com/Manual/class-Camera.html) (default: `None`)
- `Distortion Shader` - reference to [*ComputeShader*](https://docs.unity3d.com/ScriptReference/ComputeShader.html) asset about *Distortion Shader* functionality (default: `None`)
- `Ros Image Shader` - reference to [*ComputeShader*](https://docs.unity3d.com/ScriptReference/ComputeShader.html) asset about *Ros Image Shader* functionality
(default: `None`)

#### Output Data

The sensor computation output format is presented below:

|      Category      |       Type       | Description                   |
| :----------------: | :--------------: | :---------------------------- |
| *ImageDataBuffer*  |     byte[ ]      | Buffer with image data.       |
| *CameraParameters* | CameraParameters | Set of the camera parameters. |

## CameraRos2Publisher Script 
![script_ros2](script_ros2.png)
Converts the data output from `CameraSensor` to *ROS2* [Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html) 
and [CameraInfo](https://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html) type messages and publishes them. 
The conversion and publication is performed using the `Publish(CameraSensor.OutputData outputData)` method,
which is the `callback` triggered by *CameraSensor Script* for the current output.

 Due to the fact that the entire image is always published, the [`ROI`](https://docs.ros2.org/latest/api/sensor_msgs/msg/RegionOfInterest.html) field of the message is always filled with zeros. The script also ensures that `binning` is assumed to be zero and the rectification matrix is the identity matrix. 

!!! warning
    The script uses the camera parameters set in the *CameraSensor script* - remember to configure them depending on the camera you are using.

#### Elements configurable from the editor level

- `Image Topic` - the *ROS2* topic on which the [`Image`](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html) message is published<br>(default: `"/sensing/camera/traffic_light/image_raw"`)
- `Camera Info Topic` - the *ROS2* topic on which the [`CameraInfo`](https://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html) message is published<br>(default: `"/sensing/camera/traffic_light/camera_info"`)
- `Frame id` - frame in which data is published, used in [`Header`](https://docs.ros2.org/latest/api/std_msgs/msg/Header.html)<br>(default: `"traffic_light_left_camera/camera_link"`)
- `Qos Settings` - Quality of service profile used in the publication<br>(default: `Best effort`, `Volatile`, `Keep last`, `1`)

#### Published Topics

- Frequency: `10Hz`
- QoS: `Best effort`, `Volatile`, `Keep last/1`

|    Category    | Topic                                       | Message type             |               `frame_id`                |
| :------------: | :------------------------------------------ | :----------------------- | :-------------------------------------: |
| *Camera info*  | `/sensing/camera/traffic_light/camera_info` | `sensor_msgs/CameraInfo` | `traffic_light_left_camera/camera_link` |
| *Camera image* | `/sensing/camera/traffic_light/image_raw`   | `sensor_msgs/Image`      | `traffic_light_left_camera/camera_link` |
