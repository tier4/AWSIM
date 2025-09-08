# ScenarioSimulatorConnection

## Abstract
`Traffic Simulation` simulates traffic situation follow traffic rules.<br>
Spawn points (Spawnable Lanes) and spawnable vehicles can be configured using components and `Traffic Simulation` simulates traffic situation following configuration.

![Traffic Simulation](./top.png)

### Overview
`Traffic Simulation` consists of the following components:

| Component | Description |
|---|---|
| TrafficSimulator | Collecting all traffic simulators and managing the spawning and simulating process. |
| NpcVehicleSpawner | Get Npc vehicle states and updating simulation steps. |
| NpcVehicleSimulator | Spawning random Npc vehicle in spawning lanes. |
| RandomTrafficSimulator | Managing lifecycle of NPCs and simulating NPC behaviours. |
| TrafficLane<br>TrafficLight<br>TrafficIntersection<br>StopLine | Traffic entities. |
| NpcVehicle | Vehicle models (NPCs) controlled by `RandomTrafficSimulator`. |

### Configuration
`Traffic Simulation` can be configured from `TrafficSimulator` component.

The configurable elements are listed in the following table:

| Parameter | Description |
|---|---|
| Ego Vehicle | Ego vehicle handler. If not set, the manager creates a dummy ego.<br>This reference is also set automatically when the Ego spawns via the traffic simulator. |
| Seed | Seed value for random generator. |
| Traffic Intersections | The field that is set `TrafficIntersection` objects.<br>`TrafficIntersection` to be set is controlled by `Traffic Simulation`. |

## Instruction
To use `Traffic Simulation`, please follow the steps below.

For the preparation, the following must be prepared:

- 3D map (.fbx)
- lanelet map (.osm)

### 1. Placement and settings of `ScenarioSimulatorClient`
Create empty `GameObject` (should be named `TrafficSimulator`).<br>
Attach this object to `TrafficSimulator` component.

(optional) To place objects which is generated later, you may creat empty objects named `TrafficIntersections` and `NPCPedestrians`.

![Place Traffic Simulator](./hierarchy.png)

![Traffic Simulator](./traffic_simulator.png)

Please configure `TrafficSimulator` component as the following:

1. Fill in `Ego Vehicle` field with EGO vehicle
    1. Example: `EgoVehicle/Lexus RX450h 2015`
2. Change `Obstacle Layer Mask` and `Ground Layer Mask` field to `Everything`
3. Fill in `Traffic Intersections` field with `TrafficIntersection` objects
4. Fill in `Random Traffic Sims` field
    1. Fill in `Traffic Sim Npc Vehicle Prefab` field with vehicle prefabs what you want to spawn
        1. Prefabs is in `Assets/Awsim/Prefabs/Usecase/TrafficSimulation/`
    2. Fill in `Spawnable Traffic Lanes` field with `TrafficLane` where you want to spawn vehicles

!!! info
    For detailed settings of `TrafficSimulator`, see [here](#configulations).

### 2. `LaneletTrafficLight` settings
Please place intersection objects and attach `LaneletTrafficLight` script.<br>
Then, please you set traffic lights to intersection.

![Traffic Intersection](./traffic_intersection.png)

Please configure `TrafficIntersection` and `LaneletTrafficLight` components as the following:

1. Add a empty `GameObject` as a child object of `TrafficIntersections` hierarchy
    1. The `GameObject` should be named `TrafficIntersection.x`
    2. The `Transform` of `GameObject` should be set on the target intersection
2. Attach a `TrafficIntersection` component to the intersection game object
3. Attach `LaneletTrafficLight` to traffic light objects placed on the target intersection
![Traffic Light](./traffic_light.png)
4. Modify `Bulb Material Config` as follow images<br>
vehicle raffic light<br>
![Bulb Vehicle](./bulb_vehicle.png)<br>
pedestrian traffic light<br>
![Bulb Pedestrian](./bulb_pedestrian.png)
    1. If there are wrong order of bulb, modify each `Bulb Material Config`
5. Set traffic light objects attached `LaneletTrafficLight` in step. 3 to `TrafficLightGroups` in `TrafficIntersection`
    1. Traffic lights which should light same sequences should be set on same `TrafficLightGroups`

!!! warning
    Do not attach `LaneletTrafficLight` to traffic lights which are not set to `TrafficIntersection`.<br>
    It causes errors.

!!! info
    For detailed settings of `Bulb Material Config`, see [here](../../Entity/Infra/TrafficLight/index.md).

### 3. Load lanelet
`LaneletLoader` can load a lanelet map and create `TrafficLane` and `StopLine`.
In addition, `LaneletLoader` set parameter of traffic rules to `TrafficLane`, `StopLine` and traffic lights.<br>
`LaneletLoader` can be performed by opening `AWSIM -> Random Traffic -> Load Lanelet` at the toolbar of Unity Editor.

![Tool Bar](./load_lanelet_tool_bar.png)

Please use `LaneletLoader` as the following:
1. Fill in `Osm` field with a lanelet map (`.osm`) you prepared, `Root Object` field with a `TrafficSimulator` object.<br>
2. Adjust `Waypoint settings` parameters for the loading process if needed.<br>
3. To load the lanelet map, please click `Load` button.

![Load Lanelet](./load_lanelet.png)

The `Waypoint settings` parameters are listed in the following table:

| Parameter | Description |
|---|---|
| Resolution | Resolution of resampling. Lower values provide better accuracy at the cost of processing time. |
| Min Delta Length | Minimum length(m) between adjacent points. |
| Min Delta Angle | Minimum angle(deg) between adjacent edges.<br>Lowering this value produces a smoother curve. |

`TrafficLane`, `StopLine` and `TrafficLight` will be generated and placed as child objects of the `Root Object`.<br>
You can check their visual representation by clicking consecutive elements in the scene hierarchy.

![Hierarchy](./hierarchy2.png)

### 4. Call methods of `ScenarioSimulatorClient` and `ClockRos2Publisher`
To enable `Traffic Simulation`, some methods of `TrafficSimulator` should be called from callback of `MonoBehaviour`.

Please create or open class which is inherit `MonoBehaviour` and make field of `TrafficSimulator`.<br>
Then, add description of calling method of `TrafficSimulator`.

The method should be called are listed in the following table:

| Method | Description |
|---|---|
| Initialize() | Should be called Start() callback. |
| OnUpdate() | Should be called Update() callback. |
| OnFixedUpdate() | Should be called FixedUpdate() callback. |

!!! info
    AWSIM includes `AutowareSimulationDemo` scene.<br>
    Please refer to:<br>
    * `Assets/Awsim/Scenes/AutowareSimulationDemo/AutowareSimulationDemo.cs`<br>
    * `Assets/Awsim/Scenes/AutowareSimulationDemo.unity` scene

