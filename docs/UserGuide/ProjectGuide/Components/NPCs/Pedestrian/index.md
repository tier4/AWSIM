
# NPC Pedestrian

`NPCPedestrian` is an object that simulates a human moving on the stage. It can move cyclically in any chosen place thanks to the available scripts. Traffic light tracking will be implemented in the future.

<img src=model.png width=600px>

!!! note "Sample scene"
    If you would like to see how `NPCPedestrian` works or run some tests, we encourage you to familiarize yourself with the `NPCPedestrianSample` default scene described in this [section](../../../DefaultExistingScenes/).

#### Prefab and `*.fbx`

Prefab can be found under the following path:<br>
`Assets\AWSIM\Prefabs\NPCs\Pedestrians\humanElegant.prefab`

<img src=prefab.png width=600px>

Prefab is developed using models available in the form of `*.fbx` file.
From this file, the visual elements of the model, `Animator` and `LOD` were loaded.
The `Animator` and `LOD` are added as components of the main-parent *GameObject* in prefab, while the visual elements of the model are added as its children.

`*.fbx` file can be found under the following path:<br>
`Assets/AWSIM/Models/NPCs/Pedestrians/Human/humanElegant.fbx`
<img src=prefab_link.png width=300px>

#### Link

<img src=link.png width=300px>

#### Scripts and components
`NPCPedestrian.cs` script that controls the pedestrian's behavior are placed under `AWSIM\Assets\AWSIM\Scripts\NPCs\Pedestrians` directory.


## RightBody
<img src=rightbody.png width=500px>

## LOD
<img src=lod.png width=500px>

## Animator
<img src=animator.png width=500px>

## NPCPedestrian Script
<img src=script.png width=500px>

Supported features:

- Move inverse kinematically based on `Vector3 position`.
- Rotate inverse kinematically based on `Vector3 rotation`.


#### Output

| API         | type    | feature                                                                                             |
| :---------- | :------ | :-------------------------------------------------------------------------------------------------- |
| SetPosition | Vector3 | Move the NPC pedestrian so that the reference point is at the specified coordinates.                |
| SetRotation | Vector3 | Rotate the NPC pedestrian so that the orientation of the reference point becomes the specified one. |


## SimplePedestrianWalkerController Script
<img src=simple_walk.png width=500px>

<video width="1920" controls>
    <source src="walk.mp4" type="video/mp4">
</video>

## Collider


!!! example "Capsule Collider"
    Capsule collider
    
    <img src=collider.png width=500px>
    <img src=collider_example.png width=500px>







