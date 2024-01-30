## Introduction

The `FollowCamera` component is designed to track a specified target object within the scene. It is attached to the main camera and maintains a defined distance and height from the target. Additionally, it offers the flexibility of custom rotation around the target as an optional feature.

#### Elements configurable from the editor level

![main_camera_inspector](main_camera_inspector.png)

##### Required member

- `Target` - the transform component of the tracked object

##### Base Setttings

- `Distance` - base distance between the camera and the target object
- `Offset` - lateral offset of the camera position
- `Height` - base height of the camera above the target object
- `HeightMultiplier` - camera height multiplier

##### Optional Movement Setttings

- `RotateAroundModeToggle` - toggle key between rotate around mode and default follow mode
- `RotateAroundSensitivity` - mouse movement sensitivity for camera rotation around the target
- `HeightAdjustmentSensitivity` - mouse movement sensitivity for camera height adjustment
- `ZoomSensitivity` - mouse scroll wheel sensitivity for camera zoom
- `InvertHorzAxis` - invert horizontal mouse movement
- `InvertVertAxis` - invert vertical mouse movement
- `InvertScrollWheel` - invert mouse scroll wheel
- `MaxHeight` - maximum value of camera height
- `MinDistance` - minimum value of camera distance to target object
- `MaxDistance` - maximum value of camera distance to target object

#### Rotate Around Mode

Camera rotation around the target can be activated by pressing the `RotateAroundModeToggle` key (default `'C'` key). In this mode, the user can manually adjust the camera view at run-time using the mouse. To deactivate the Rotate Around mode, press the `RotateAroundModeToggle` key once more.

In the Rotate Around Mode camera view can be controlled as follows:

- `mouse movement`: to adjust the camera position,
- `mouse scroll wheel`: to zoom in or out of the camera view.


#### Optional
An optional prefab featuring a UI panel, located at `Assets/Prefabs/UI/MainCameraView.prefab`, can be used to showcase a user guide. To integrate this prefab into the scene, drag and drop it beneath the Canvas object. This prefab displays instructions on how to adjust the camera view whenever the Rotate Around Mode is activated.
