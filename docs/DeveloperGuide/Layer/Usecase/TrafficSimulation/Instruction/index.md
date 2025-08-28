# Instruction
To enable `Traffic Simulation`, please follow the steps below.

For the preparation, the following must be prepared:

- 3D map
- lanelet map (.osm)

## Environment (3D Map) setting

## TrafficLight setting
Please attach `LaneletTrafficLight` component to all traffic light included in `3D Map`.

[img]

## Load lanelet
`LaneletLoader` can load lanelet and set parameter of traffic rules to `TrafficLane`, `StopLine` and `TrafficLight`.<br>
`LaneletLoader` can be performed by opening `AWSIM -> Random Traffic -> Load Lanelet` at the toolbar of Unity Editor.

[img]

Please fill in `Osm` field with lanelet map (`.osm`) you prepared, in `Root Object` field with `TrafficSimulator` object.<br>
Please adjust the parameters for the loading process if needed.<br>
To load lanelet map, please click `Load` button.

[img]

The `Waypoint settings` parameters are listed in the following table:

| Parameter | Description |
|---|---|
| Resolution | Resolution of resampling. Lower values provide better accuracy at the cost of processing time |
| Min Delta Length | Minimum length(m) between adjacent points |
| Min Delta Angle | minimum angle(deg) between adjacent edges. Lowering this value produces a smoother curve |

Environment components should be generated and placed as child objects of the Environment GameObject.<br>
You can check their visual representation by clicking consecutive elements in the scene hierarchy.

## TrafficIntersection setting
Please locate intersection object and set traffic lights to it.

[img]

Please configure `TrafficIntersection` component sas the following:

1. Add a `GameObject` as a child object of the `TrafficIntersections`
    1. the `GameObject` should be named `TrafficIntersection.x`
    2. the `Transform` of `GameObject` should be set on target intersection
2. Attach a `TrafficIntersection` component to the `GameObject`
3. Set `TrafficLightGroups` to object attached `LaneletTrafficLight` component replaced on target intersection
    1. traffic lights which should light same sequences should be set on same `TrafficLightGroups`
4. (optional) Modify the signal control pattern in `Lighting Sequences`

## TrafficSimulator setting

[img]

Please configure the `TrafficSimulator` component as the following:

1. Fill in the `Traffic Intersections` field with `TrafficIntersection` objects
2. Fill in `Random Traffic Sims` field
    1. Fill in `Traffic Sim Npc Vehicle Prefab` field with vehicle prefab what you want to spawn
    2. Fill in `Spawnable Traffic Lanes` field with `TrafficLane` where you want to spawn vehicles

For detailed settings, see [here](../Abstract/index.md#configulations)

## Locate Pedestrian (optional)
You can locate pedestrian NPC if you need.<br>
Pedestrian can animated and walk around where they are located.<br>
Direction which pedestrian start to walking can be set in `Transform` of it.

[img]

Please configure Pedestrian NPCs component as the following:

1. Locate pedestrian prefab on scene
    1. Prefab is in `Assets/Awsim/Prefabs/Entity/Npc/Pedestrian/`
    2. Pedestrian should be child object of `NPCPedestrians` hierarchy
2. Attach `SimplePedestrianWalkerController` component to pedestrian
3. (optional) Configure parameters of `SimplePedestrianWalkerController`

The parameters of `SimplePedestrianWalkerController` are listed in the following table:

| Parameter | Description |
|---|---|
| Duration | Duration in which the pedestrian animate |
| Speed | Speed at which the pedestian walk |

## Reference Components
To enable `Traffic Simulation`, please fill in following fields in `AutowareSimulationDemo.cs`.

[img]

### TrafficSimulator
Please fill in `Traffic Simulator` field in `AutowareSimulationDemo.cs` with a object which is attached `TrafficSimulator.cs`.

### NPCPedestrians (optional)
Please fill in `Simple Pedestrian Walker Controller` list in  `AutowareSimulationDemo.cs` with `Pedestrian` if you located.