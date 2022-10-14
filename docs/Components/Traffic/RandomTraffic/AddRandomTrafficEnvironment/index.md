# Add Environment for Random Traffic
This page describes the steps to add environment components required by `RandomTrafficSimulator`.

## Prepare Map
Add 3d model map to be annotated to the scene. Make sure that an `Environment` component with appropriate `mgrsOffsetPosition` is attached to the root GameObject.
![](map.png)  

## Annotate Traffic Lights
Attach `TrafficLight` component to all traffic light game objects.
![](traffic_light.png)  

## Load Lanelet
Open `AWSIM -> Random Traffic -> Load Lanelet`.
![](load_lanelet.png)

Set OSM file and change settings as needed. 
![](lanelet_loader_window.png)  
Waypoint settings affect the density and accuracy of the waypoints generated.
- Resolution
  - Resolution of resampling. Lower values provide better accuracy at the cost of processing time.
- Min Delta Length
  - Minimum length(m) between adjacent points.
- Min Delta Angle
  - Minimum angle(deg) between adjacent edges. Lowering this value produces a smoother curve. Click the Load button. Environment components are generated.

Click `Load` button. `TrafficLane` components and `StopLine` objects should be placed as child objects of Environment game object. You can check them in a editor view by selecting them.
![](environment_components.png)  

## Annotate Traffic Intersections
![](traffic_intersection.png)  

Add an empty GameObject named `TrafficIntersections` in the same hierarchy as the TrafficLanes object.

For each intersection, do the following:
1. Add an object named TrafficIntersection as a child object of the TrafficIntersections object.
Attach a TrafficIntersection component.
2. Set BoxCollider to cover the intersection. This is used for detecting vehicles in the intersection.
3. Set TrafficLightGroups. Each group is controlled to have different signals, so facing traffic lights should be added to the same group. These groupings are used in traffic signal control.
4. Specifies the signal control pattern.

## Annotate Additional Right of Ways
You need to annotate right of ways of `TrafficLane` manually for unsignalized intersections.

![](select_traffic_light.png)  
Select straight lane that is not right of way in the intersection. The selected lane should be highlighted.(The image is for illustration.)
![](set_right_of_way.png)

Click `Set RightOfWays` button.

![](right_of_ways.png)  

Check all lanes that intersect the lane are highlighted yellow.(The image is for illustration.)

## Annotate Additional Stop Lines
For each right turn lane that yields to the opposite straight or left turn lane, a stop line needs to be defined near the center of the intersection.
![](stop_lines.png)  
If there is no visible stop line, a `StopLine` component should be added near the center of the intersection and associated with `TrafficLane`.

## Run and Check Random Traffic
Once all the components are ready, let's get the random traffic moving once.
For each intersection, review the settings of the relevant components if vehicles are unable to proceed.
