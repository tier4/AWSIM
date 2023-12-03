# Random Traffic Simulator
The `RandomTrafficSimulator` simulates city traffic with respect to all traffic rules. The system allows for random selection of car models and the paths they follow. It also allows adding static vehicles in the simulation.

![random_traffic](random_traffic.png)

## Getting Started
### Overview

The random traffic system consists of the following components:

- `RandomTrafficSimulator`:  manages lifecycle of NPCs and simulates NPC behaviours.
- `TrafficLane`, `TrafficIntersection` and `StopLine`: represent traffic entities
- `NPCVehicle`: vehicle models (NPCs) controlled by `RandomTrafficSimulator`

![](overview.png)

## Components Settings

The following section describes Unity Editor components settings.

### Random Traffic Simulator

![inspector](inspector.png)

| Parameter | Description |
|---|---|
| **General Settings** | |
| Seed | Seed value for random generator |
| Ego Vehicle | Transform of ego vehicle |
| Vehicle Layer Mask | LayerMask that masks only vehicle(NPC and ego) colliders |
| Ground Layer Mask | LayerMask that masks only ground colliders of the map |
| **NPC Vehicle Settings** | |
| Max Vehicle Count | Maximum number of NPC vehicles to be spawned in simulation|
| NPC Prefabs | Prefabs representing controlled vehicles.<br/> They must have `NPCVehicle` component attached. |
| Spawnable Lanes | `TrafficLane` components where NPC vehicles can be spawned during traffic simulation|
| Vehicle Config | Parameters for NPC vehicle control<br/>`Sudden Deceleration` is a deceleration related to emergency braking |
| **Debug** | |
| Show Gizmos | Enable the checkbox to show editor gizmos that visualize behaviours of NPCs |

## Gizmos
Gizmos are useful for checking current behavior of NPCs and its causes.
Gizmos have a high computational load so please disable them if the simulation is laggy.
![gizmos](gizmos.png)
