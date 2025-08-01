AWSIM's architecture is designed to meet the requirements of various automated driving simulations.
It is designed to be easy to use with key components in a commonly use and with customizability for niche requirements.

- Layered architecture is used, with dependencies built at the namespace level.
- Except for a single scene class within a scene, MonoBehaviour scripts within AWSIM do not use the Unity callbacks for `Awake()`, `Start()`, `Update()`, and `FixedUpdate()`.
    - Instead, Public methods such as `Initialize()` and `OnUpdate()`, `OnFixedUpdate()` are called from the scene class.

## Layer


AWSIM uses a layered architecture. Dependencies are built on the following layers. The layers are separated by the root namespace.

``` mermaid 
flowchart LR
    Scene --> UI
    UI --> Usecase
    Usecase --> Entity
    Entity --> Common
```

!!! warning
    It is a unilateral dependence from left to right. For example, a `Usecase` layer can depend on a `Common` layer, but not vice versa.

|Layer|Root namespace|Note|Mainly features|
|:--|:--|:--|:--|
|Scene|Awsim.Scene|**The scene layer contains the implementation for that particular scene.**<br><br> Basically, one namespace per scene is supported. (For example, in `AutowareSimulationDemo.unity` scene is `Awsim.Scene.AutowareSimulationDemo`).<br> One scene class is assumed to be created for each scene. This scene class will be the entry point and main loop.<br> Features commonly used in multiple scenes are not included in this layer and are not intended for reuse.<br>The Scene layer scripts are used only for each scene, so they are placed under `Assets\Awsim\Scenes`, not under `Assets\Awsim\Scirpts`.|･ Scene|
|UI|Awsim.UI|**The UI layer provides a unified UI used by AWSIM.**<br><br> Basically, all UIs inherit from the `UIWIndow` class and support dragging to move, hide, and scale. UI does not include critical logic in the simulation. Focus on user input and information display.|･ UI|
|Usecase|Awsim.Usecase|**The Usecase layer has upstream implementations that depend on Entity.**<br><br>This layer will aggregate functions according to the purpose of each use case. Basically, it is a combined implementation of Entity.|･ Create pcd<br> ･ Traffic<br> ･ Scenario|
|Entity|Awsim.Entity|**The Entity layer provides essential components for automated driving simulation.**<br><br> Entity layer provides components such as vehicles, sensors, infrastructure and pedestrians. These are supposed to be used in combination depending on the simulation content. Basically, initialization and updating is done from higher layers.|･ Vehicle<br> ･ Pedestrian<br> ･ Sensor<br> ･ Infra<br>|
|Common|Awsim.Common|**The Common layer has the most primitive implementations.**<br><br> For example, Math, Time, ROS2-related, etc. This layer is independent of the context of automated driving and provides generic functionality. The Common layer is never dependent on any other layer.|･ ROS2<br> ･ Geo<br> ･ Math<br> ･ Time<br> ･ Lanelet<br> ･ etc.|

## AWSIM Namespace

|Namespace|Feature|
|:--|:--|
|`Awsim.Scene.AutowareSimulationDemo`|Script used only in `AutowareSimulationDemo.unity` scene.|
|`Awsim.Scene.IntegrateScenarioSimulatorDemo`|Script used only in `IntegrateScenarioSimulatorDemo.unity` scene.|
|`Awsim.Scene.PcdGenerationDemo`|Script used only in `PcdGenerationDemo.unity` scene.|
|`Awsim.UI`|UI scripts.|
|`Awsim.Usecase.PcdGeneration`|Script for pcd generation.|
|`Awsim.Usecase.ScenarioSimulatorConnection`|Script for scenario simulator v2 connection.|
|`Awsim.Usecase.TrafficSimulation`|Script for traffic simulation.|
|`Awsim.Entity`|Scripts included in the `Entity` layer.|
|`Awsim.Common`|Scripts inlluded in the `Common` layer.|


## Scene class

In AWSIM, the concept of scene classes is one of the most important architectural concepts. For example, in the `AutowareSimulationDemo.unity` scene, the scene class is [AutowareSimulationDemo.cs]().

Creating a scene class for each scene has the following advantages.

- The entry point for the program is clear. Basically, you can understand the flow of the process by simply reading the scene class from the top.
- Execution order can be controlled at the C# coding level without using [Script Execution Order]().
- Initialization and loop processing can be written to suit the scene. For example, if the startup Json configurations differ from scene to scene, creating a scene class for each scene allows for flexible customization.
- Object creation and destruction, will be managed with the scene class as the top level, making it easier to manage the lifecycle of each instance.
- Basically, the scene class has a reference by `[SerializeField]` to the objects in the scene, making it easier to prevent inspector reference explosions.