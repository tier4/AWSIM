AWSIM has a unified UI system. Also, the UI does not contain critical logic. For this reason, it is ranked higher among Layers. It AWSUN UI system created based on [UGUI](https://docs.unity3d.com/Packages/com.unity.ugui@2.0/manual/index.html).

The following features are available.

- Moving UI windows by drag & drop
- UI Scale Adjustment
- On/Off for each UI window
- UI window position memorization and resetting
- Extensibility-aware UI base class

## UI Core components

|Class|Feature|
|:--|:--|
|AwsimCanvas|This is the core class of AWSIM's UI system. It has a reference to each UI class and handles all UI functions centrally. |
|UIWindow|Base class for each UIWindow. Custom UI classes can inherit from this class to easily utilize the functionality of the AWSIM UI system.|

## Create custom UI

1. Create a custom UI class that extends the `UIWindow` class
1. Create a gameobject with a custom UI class.
1. Place `AwsimCanvas.prefab` in the scene. (path : `Asstes\Awsim\Prefabs\UI\AwsimCanvas\AwsimCanvas.prefab`)
1. Place gameobject with custom UI class under `AwsimCanvas/UIWindows` in the hierarchy.
1. Referencing a custom UI class in the `UiWindows` field of AwsimCanvas.