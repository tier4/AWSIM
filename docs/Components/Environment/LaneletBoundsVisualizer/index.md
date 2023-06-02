# Lanelet Bounds Visualizer

Lanelet Bounds Visualizer is an Unity Editor extension allowing the user to load the left and right bounds of Lanelet to the Unity scene.

## Usage

The lanelet bounds load process can be performed by opening `AWSIM -> Visualize -> Load Lanelet Bounds` at the top toolbar of Unity Editor.

<img src=image_0.png>

A window shown below will pop up. Select your `Osm Data Container` to specify which OSM data to load the Lanelet from.

<img src=image_1.png>

The user can select whether to load the raw Lanelet or to adjust the resolution of the Lanelet by specifying the waypoint settings.

To load the raw Lanelet, simply click the `Load Raw Lanelet` button.

If the user wishes to change the resolution of the Lanelet, adjust the parameters of the `Waypoint Settings` as described below, and click the `Load with Waypoint Settings` button.

- `Resolution` : resolution of resampling. Lower values provide better accuracy at the cost of processing time.
- `Min Delta Length` : minimum length(m) between adjacent points.
- `Min Delta Angle` : minimum angle(deg) between adjacent edges. Lowering this value produces a smoother curve.


Once the Lanelet is successfully loaded, Lanelet bounds will be generated as a new GameObject named `LaneletBounds`.

To visualize the `LaneletBounds`, make sure Gizmos is turned on and select the `LaneletBounds` GameObject.
<img src=image_2.png>


## Important Notes

Generally speaking, visualizing Lanelet Bounds will result in a very laggy simulation. Therefore, it is recommended to hide the `LaneletBounds` GameObject when not used. The lag of the simulation becomes worse as you set the resolution of the Lanelet Bounds higher, so it is also recommended to set the resolution within a reasonable range.

It is also important to note that no matter how high you set the resolution to be, it will not be any better than the original Lanelet (i.e. the raw data). Rather, the computational load will increase and the simulation will become more laggy. If the user wishes to get the highest quality of Lanelet Bounds, it is recommended to use the `Load Raw Lanelet` button.

In short, `Waypoint Setting` parameters should be thought of as parameters to decrease the resolution from the original Lanelet to decrease the computational load and thus, reducing the lag of the simulation.


| Higher Resolution                | Raw Lanelet		              | Lower Resolution          		 |
| -------------------------------- | -------------------------------- | -------------------------------- |
|<img src=image_3.png width=250px> |<img src=image_4.png width=250px> |<img src=image_5.png width=250px> |
