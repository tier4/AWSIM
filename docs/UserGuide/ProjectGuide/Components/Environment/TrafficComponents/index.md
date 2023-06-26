# TrafficComponents
This is a section that describes in detail all objects related to simulated traffic in the `Environment` prefab.

The random traffic system consists of the following components:

- `RandomTrafficSimulator`:  manages lifecycle of NPCs and simulates NPC behaviours.
- `TrafficLane`, `TrafficIntersection` and `StopLine`: represent traffic entities
- `NPCVehicle`: vehicle models (NPCs) controlled by `RandomTrafficSimulator`

![overview](overview.png)

## Lanelet2
*Lanelet2* is a library created for handling a map focused on automated driving.
It also supports ROS and ROS2 natively.
In *AWSIM* lanelet2 is used for reading and handling a map of all roads.
You may also see us referring to the actual map data file (`*.osm`) as a *lanelet2*.

!!! info
    If you want to learn more we encourage to visit the [official project page](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/tree/master).

## Random Traffic Simulator
The `RandomTrafficSimulator` simulates city traffic with respect to all traffic rules. The system allows for random selection of car models and the paths they follow. It also allows adding static vehicles in the simulation.

#### Link
The `RandomTrafficSimulator` consists of several *GameObjects*.

- `RandomTrafficSimulator` - this is an *Object* consisting of a `Traffic Manager` Script.<br>
    You can learn more about it [here](#traffic-intersection-script).
- `TrafficIntersections` - this is a parent *Object* for all `Traffic Lanes`.<br>
    You can learn more about it [here](#trafficintersections).
- `TrafficLanes` - this is a parent *Object* for all `Traffic Lanes`.<br>
    You can learn more about it [here](#trafficlanes).
- `StopLines` - this is a parent *Object* for all `Stop Lines`.<br>
    You can learn more about it [here](#stoplines).

![random_traffic_link](random_traffic_link.png)

#### Traffic Manager Script
The `Traffic Manager` Script is responsible for all of top level management of the [NPC Vehicles](../../NPCs/Vehicle/).
It does spawn NPC Vehicles on the Traffic Lanes.
This Script also configures all of the spawned NPC Vehicles, so that they all have common parameters.

`Traffic Manager` Script can operate in two modes separately or simultaneously.

![random_traffic_script](random_traffic_script.png)

##### Random Traffic 
In the Random Traffic mode the NPC Vehicle prefabs (*NPC Prefabs*) can be chosen as well as *Spawnable Lanes*.
The later are the only Traffic Lanes on which the NPC Vehicles can spawn.
After that the NPC Vehicle takes a random route until it drives out of the map - then it is destroyed.

![random_traffic_sims](random_traffic_sims.png)

##### Route Traffic
In the Route Traffic mode one of the NPC Vehicle prefabs (*NPC Prefabs*) spawns on the first Route Element (`Element 0`) and then drives on the specified route.
After the first vehicle drove off the next one spawns according to the configuration.
It is **important** for all *Route* section elements to be connected an to be arranged in order of appearance on the map.
The NPC Vehicle disappears after completing the Route.

Many different *Routes* can be added and they will operate independently.

![route traffic sims](route_traffic_sims.png)

#### Parameter explanation
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
<!-- TODO -->
![intersection](intersections/intersection.png)

#### Link
<!-- TODO -->
![intersections_link](intersections/intersections_link.png)

#### Prefab
<!-- TODO -->
![intersection_prefab](intersections/intersection_prefab.png)

#### Traffic Intersection Script
<!-- TODO -->
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
<!-- TODO -->
![intersection_collider](intersections/intersection_collider.png)

## TrafficLanes
<!-- TODO -->
![lanes](traffic_lanes/lanes.png)

#### Link
<!-- TODO -->
![lanes_link](traffic_lanes/lanes_link.png)

#### Prefab
<!-- TODO -->
![lanes_prefab](traffic_lanes/lanes_prefab.png)

#### Traffic Lane Script
<!-- TODO -->
![lanes_script](traffic_lanes/lanes_script.png)

Right Of Way Lanes

![lanes_yield](traffic_lanes/lanes_yield.png)

![lanes_yield_2](traffic_lanes/lanes_yield_2.png)



## StopLines
<!-- TODO -->
![stop](stop_lines/stop.png)

![stop_lines_link](stop_lines/stop_lines_link.png)

#### Link
<!-- TODO -->

#### Prefab
<!-- TODO -->
![stop_prefab](stop_lines/stop_prefab.png)

![stop_script](stop_lines/stop_script.png)

## Gizmos
<!-- TODO -->
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


