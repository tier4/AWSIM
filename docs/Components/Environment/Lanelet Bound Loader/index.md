# Lanelet Bound Loader

Lanelet Bound Loader is an Unity Editor extension enabling the user to load the left and right sides of Lanelet to the Unity scene.

## Usage

The lanelet bounds load process can be performed by opening `AWSIM -> Random Traffic -> Load Lanelet Bounds` at the top toolbar of Unity Editor.

<img src=image_0.png>

A window shown below will pop up. Select your `Osm Data Container` to specify which lanelet to load and adjust the parameters as needed.

<img src=image_1.png>

The parameters are described below:

- Resolution: resolution of resampling. Lower values provide better accuracy at the cost of processing time.
- Min Delta Length: minimum length(m) between adjacent points.
- Min Delta Angle: minimum angle(deg) between adjacent edges. Lowering this value produces a smoother curve.

Finally, click the `Load` button to generate the sides of the Lanelet. The Lanelet bounds will be generated as a new GameObject named `TrafficLanesBound`.

| Lanelet without bounds  		   | Lanelet with bounds 		       |
| -------------------------------- | --------------------------------- |
|<img src=image_2.png width=375px> | <img src=image_3.png width=375px> |
