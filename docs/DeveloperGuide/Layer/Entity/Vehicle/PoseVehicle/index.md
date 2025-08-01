
`PoseVehicle` is pose (position and rotation) input based vehicle. This vehicle model is mainly used to control **NPC vehicles**. Controlling vehicles through pose input enables more versatile traffic simulation.

!!! info
    Ego vehicle mainly uses acceleration-based input for vehicle dynamics. For details, [here](../AccelVehicle/index.md).

## Prefab

Path : `Assets\Awsim\Prefabs\Entity\Npc\Vehicle\*`

|              |        Hatchback         |         SmallCar         |           Taxi           |          Truck           |           Van            |
| :----------: | :----------------------: | :----------------------: | :----------------------: | :----------------------: | :----------------------: |
| *Appearance* | ![vehicles](image_1.png) | ![vehicles](image_2.png) | ![vehicles](image_3.png) | ![vehicles](image_4.png) | ![vehicles](image_5.png) |
|   *Prefab*   |    `Hatchback.prefab`    |    `SmallCar.prefab`     |     `Taxi-64.prefab`     |    `Truck_2t.prefab`     |       `Van.prefab`       |

## Physics engine effect

`PoseVehicle` improves realism by allowing the physics engine to affect certain axes.

|Axis|Note|
|:--|:--|
|Position x|input value.|
|Position y|Physics engine.|
|Position z|input value.|
|Rotation x|input value.|
|Rotation y|input value.|
|Rotation z|Physics engine.|

## How to use

1. Attach the `PoseVehicle` script to the vehicle game object and set up each item.
1. Call `PoseVehicle.Initialize()` during initialization.
1. Enter a value in `PoseVehicle.PoseInput` before calling `OnFixedUpdate()`.
1. Call `OnUpdate()` in `Update()` and `OnFixedUpdate()` in `FixedUpdate()`.