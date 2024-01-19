# Add Environment for Random Traffic

This document describes the steps to properly configuer `RandomTrafficSimulator` in your environment.

## Map preparation

The 3D map model should be added to the scene. Please make sure that the `Environment` component with appropriate `mgrsOffsetPosition` is attached to the root GameObject.
![](map.png)

## Annotate Traffic Lights

Please attach `TrafficLight` component to all traffic light GameObjects placed on scene.
![](traffic_light.png)  

## Load Lanelet

The lanelet load process can be performed by opening `AWSIM -> Random Traffic -> Load Lanelet` at the top toolbar of Unity Editor.
![](load_lanelet.png)

You should be prompted with a similar window to the one presented below. Please adjust the parameters for the loading process if needed.

![](lanelet_loader_window.png)

Waypoint settings affect the density and accuracy of the generated waypoints. The parameters are described below:

- Resolution: resolution of resampling. Lower values provide better accuracy at the cost of processing time.
- Min Delta Length: minimum length(m) between adjacent points.
- Min Delta Angle: minimum angle(deg) between adjacent edges. Lowering this value produces a smoother curve.

To generate the Lanelet2 map representation in your simulation, please click the `Load` button. Environment components should be generated and placed as child objects of the `Environment` GameObject. You can check their visual representation by clicking consecutive elements in the scene hierarchy.

![](environment_components.png)

## Annotate Traffic Intersections
![](traffic_intersection.png)

To annotate intersection please, add an empty GameObject named `TrafficIntersections` at the same level as the `TrafficLanes` GameObject.

For each intersection repeat the following steps:

1. Add an GameObject named `TrafficIntersection` as a child object of the `TrafficIntersections` object.
2. Attach a `TrafficIntersection` component to it.
3. Add a  `BoxCollider` as a component of GameObject. It's size and position should cover the whole intersection. This is used for detecting vehicles in the intersection.
4. Set `TrafficLightGroups`. Each group is controlled to have different signals, so facing traffic lights should be added to the same group. These groupings are used in traffic signal control.
5. Specify the signal control pattern.

## Annotate right of ways on uncontrolled intersections

For the vehicles to operate properly it is needed to annotate the right of way of `TrafficLane` manually on intersections without traffic lights.


![](select_traffic_light.png)

To set the right of way, please:

- Select a straight lane that is not right of way in the intersection. The selected lane should be highlighted as presented below.
- Click the `Set RightOfWays` button to give the lane priority over other lanes.
![](set_right_of_way.png)
- Please check if all lanes that intersect with the selected lane are highlighted yellow. This means that the right of way was applied correctly.
![](right_of_ways.png)

## Annotate stop lines

For each right turn lane that yields to the opposite straight or left turn lane, a stop line needs to be defined near the center of the intersection.
![](stop_lines.png)
If there is no visible stop line, a `StopLine` component should be added to the scene, near the center of the intersection and associated with `TrafficLane`.

## Assign Intersection TrafficLanes
To make the yielding rules work properly, it is necessary to catagorize the `TrafficLanes`. 
The ones that belong to an intersection have the `IntersectionLane` variable set to true.

To automate the assignment of the corresponding `IntersectionLane` to each `TrafficLane`, the script `AssignIntersectionTrafficLanes` can be used.
![](intersectionlane_script.png)

1. At the time of assignment, add it as a component to some object in the scene (e.g. to the `Environment` object).
2. Disable the component (uncheck the checkbox next to the script name).
3. Assign to `TrafficLanesObjectsParent` *GameObject*, which contains all `TrafficLanes` objects.
4. Check all 4 options.
5. Enable the component (check the checkbox next to the script name).

Check the log to see if all operations were completed:
![](log.png)

As a result, the names of `TrafficLane` objects should have prefixes with sequential numbers and `TrafficLane` at intersections should be marked. `TrafficLanes` with `IntersectionLane` set to *True* are displayed by *Gizmos* in green color, if `IntersectionLane` is *False* their color is white.
<br>![](names.png)
![](intersectionlane.png)

## Check final configuration

Once all the components are ready, the simulation can be run.
Check carefully if the vehicles are moving around the map correctly.
For each intersection, review the settings of the relevant components if vehicles are unable to proceed.
