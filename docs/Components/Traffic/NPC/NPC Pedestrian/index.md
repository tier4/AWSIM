# NPC Pedestrian

The `NPC Pedestrian` is a non-playable humanoid object and used for simulation traffic flows. 

<img src=image_0.png width=500px>

Prefab files are placed under the follow paths.

|Prefab|Path|
|:--|:--|
|`humanElegant.prefab`|`AWSIM\Assets\AWSIM\Prefabs\NPCs\Pedestrians`|
|`humanElegantTest.prefab`|`AWSIM\Assets\AWSIM\Scenes\Samples\NPCPedestrianSample`|

Supported features:

- Move inverse kinematically based on `Vector3 position`.
- Rotate inverse kinematically based on `Vector3 rotation`.

## Sample scene

The sample scene that simulates the behavior of NPC Pedestrian is placed under `AWSIM\Assets\AWSIM\Scenes\Samples` directory.

## NPC Pedestrian control scripts

`NPCPedestrian.cs` script that controls the pedestrian's behavior are placed under `AWSIM\Assets\AWSIM\Scripts\NPCs\Pedestrians` directory.


## NPC Pedestrian API

The following section describes the API of `NPCPedestrian.cs` script.

|API|type|feature|
|:--|:--|:--|
|SetPosition|Vector3|Move NPC pedestrian so that the reference point is at the specified coordinates.|
|SetRotation|Vector3|Rotate NPC pedestrian so that the orientation of the reference point becomes the specified one.|



