# TrafficComponents
This is a section that describes in detail all objects related to simulated traffic in the `Environment` prefab.

The random traffic system consists of the following components:

- `RandomTrafficSimulator`:  manages lifecycle of NPCs and simulates NPC behaviours.
- `TrafficLane`, `TrafficIntersection` and `StopLine`: represent traffic entities
- `NPCVehicle`: vehicle models (NPCs) controlled by `RandomTrafficSimulator`

![](overview.png)

## Lanelet2

## Random Traffic Simulator
The `RandomTrafficSimulator` simulates city traffic with respect to all traffic rules. The system allows for random selection of car models and the paths they follow. It also allows adding static vehicles in the simulation.

#### Link
![random_traffic_link](random_traffic_link.png)

#### Traffic Manager Script
![random_traffic_script](random_traffic_script.png)
![random_traffic_sims](random_traffic_sims.png)

| Parameter                | Description                                                                                                 |
| ------------------------ | ----------------------------------------------------------------------------------------------------------- |
| **General Settings**     |                                                                                                             |
| Seed                     | Seed value for random generator                                                                             |
| Ego Vehicle              | Transform of ego vehicle                                                                                    |
| Vehicle Layer Mask       | LayerMask that masks only vehicle(NPC and ego) colliders                                                    |
| Ground Layer Mask        | LayerMask that masks only ground colliders of the map                                                       |
| Culling Distance         | Distance at which NPCs are culled relative to EgoVehicle                                                    |
| Culling Hz               | Culling operation cycle                                                                                     |
| **NPC Vehicle Settings** |                                                                                                             |
| Max Vehicle Count        | Maximum number of NPC vehicles to be spawned in simulation                                                  |
| NPC Prefabs              | Prefabs representing controlled vehicles.<br/> They must have `NPCVehicle` component attached.              |
| Spawnable Lanes          | `TrafficLane` components where NPC vehicles can be spawned during traffic simulation                        |
| Vehicle Config           | Parameters for NPC vehicle control<br/>`Sudden Deceleration` is a deceleration related to emergency braking |
| **Debug**                |                                                                                                             |
| Show Gizmos              | Enable the checkbox to show editor gizmos that visualize behaviours of NPCs                                 |

## TrafficIntersections
![intersection](intersections/intersection.png)

#### Link
![intersections_link](intersections/intersections_link.png)

#### Prefab
![intersection_prefab](intersections/intersection_prefab.png)

#### Traffic Intersection Script
![intersection_script](intersections/intersection_script.png)

Traffic Lights Groups

![light_groups](intersections/light_groups.png)

Lighting Sequences

![lights_sequence](intersections/lights_sequence.png)

<details>
    <summary>Lighting Sequence Sample - details</summary>
    <table>
    <tr>
        <td>Description</td>
        <td>Editor</td>
    </tr>
    <tr>
        <td>AAA</td>
        <td><img src="lights_sequence/lights_sequence_1.png" width="400"></td>
    </tr>
    <tr>
        <td>AAA</td>
        <td><img src="lights_sequence/lights_sequence_2.png" width="400"></td>
    </tr>
    <tr>
        <td></td>
        <td><img src="lights_sequence/lights_sequence_3.png" width="400"></td>
    </tr>
    <tr>
        <td></td>
        <td><img src="lights_sequence/lights_sequence_4.png" width="400"></td>
    </tr>
    <tr>
        <td></td>
        <td><img src="lights_sequence/lights_sequence_5.png" width="400"></td>
    </tr>
    <tr>
        <td></td>
        <td><img src="lights_sequence/lights_sequence_6.png" width="400"></td>
    </tr>
    <tr>
        <td></td>
        <td><img src="lights_sequence/lights_sequence_7.png" width="400"></td>
    </tr>
    <tr>
        <td></td>
        <td><img src="lights_sequence/lights_sequence_8.png" width="400"></td>
    </tr>
    <tr>
        <td></td>
        <td><img src="lights_sequence/lights_sequence_9.png" width="400"></td>
    </tr>
</table>
</details>



#### Collider
![intersection_collider](intersections/intersection_collider.png)

## TrafficLanes 
![lanes](traffic_lanes/lanes.png)

#### Link
![lanes_link](traffic_lanes/lanes_link.png)

#### Prefab
![lanes_prefab](traffic_lanes/lanes_prefab.png)

#### Traffic Lane Script
![lanes_script](traffic_lanes/lanes_script.png)

Right Of Way Lanes

![lanes_yield](traffic_lanes/lanes_yield.png)
![lanes_yield_2](traffic_lanes/lanes_yield_2.png)



## StopLines
![stop](stop_lines/stop.png)
![stop_lines_link](stop_lines/stop_lines_link.png)

#### Link

#### Prefab
![stop_prefab](stop_lines/stop_prefab.png)
![stop_script](stop_lines/stop_script.png)

## Gizmos
Gizmos are useful for checking current behavior of NPCs and its causes.
Gizmos have a high computational load so please disable them if the simulation is laggy.
![gizmos](gizmos.png)


!!! node "Draft note"
    (description of what it is and how spawned NPCs behave in the environment)

    - Architecture description [30% current] (**graph**)
    - Seed, Spawnable Lanes, Max Vehicle Count, Spawning process (description, **gif**: example of spawning on a specific line)
    - Vehicle Prefabs (**screens** - examples of vehicles and the impact of boundaries)
    - Vehicle and Ground Layer masks (impact on spawning and behavior)
    - Vehicle Config (an explanation of each and a reference to SpeedMode)
    - Simulator steps (cognition, decision, control -> SpeedMode and Yielding states, **graph**)
    - Yielding process (stop lines, **gifs**)
    - Visualization description (**screens**: yielding, SpeedMode)

    **TrafficIntersection**

    (description of what it is and where it occurs in the environment, **screen**)

    - Traffic Lights (description, bulbs, dependence of the location on the lanelet, impact on the recognition of traffic lights in Autoware, **screens**)
    - Traffic Intersection Script
        - Collider Mask (probably out of date)
        - Traffic Light Groups (what they are and the result of adding traffic lights to them)
        - Lighting Sequences (description, how it works - **gifs**)

    **StopLine**

    (description of what it is and where it occurs in the environment **screen**, impact on Random Traffic)

    - Stop Line Script
        - Points
        - Stop sign (why this association occurs)
        - Traffic light (why this association occurs)

    **TrafficLane**

    (description of what it is and where it occurs in the environment **screen**, impact on Random Traffic)

    - Traffic Lane Script
        - Waypoints (**screen**)
        - Turn Direction (**screens**)
        - Next and Prev Lanes (**screens**)
        - Right of Way Lanes (**gifs**: a few examples with explanations regarding to Random Traffic)
        - Stop Line (**screen**)
        - Speed Limit


