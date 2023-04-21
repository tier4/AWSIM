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
