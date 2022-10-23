# Camera Sensor

CameraSensor is used for Autoware traffic light recognition.

## Prefabs

Path : `Assets\AWSIM\Prefabs\Sensors\CameraSensor.prefab`

## Scripts

Path : `Assets\AWSIM\Prefabs\Sensors\CameraSensor\*`

|script|feature|
|:--|:--|
|CameraSensor.cs|Camera Sensor. Apply OpenCV distortion and encode to bgr8 format. Use ComputeShader.|
|CameraRos2Publisher.cs|Convert the data output from CameraSensor to ROS2 msg and Publish.|

## Output Data

|field|type|feature|
|:--|:--|:--|
|ImageDataBuffer|byte[ ]|Buffer with image data.|
|CameraParameters|CameraParameters|Set of the camera parameters.|

## ROS2 Publish Topics

|topic|msg|frame_id|hz|QoS|
|:--|:--|:--|:--|:--|
|camera|`/sensing/camera/traffic_light/camera_info`|`sensor_msgs/CameraInfo`|`traffic_light_left_camera/camera_link`|`10`|`Best effort`, `Volatile`, `Keep last/1`|
|camera|`/sensing/camera/traffic_light/image_raw`|`sensor_msgs/Image`|`traffic_light_left_camera/camera_link`|`10`|`Best effort`, `Volatile`, `Keep last/1`|