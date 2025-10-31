`AccelVehicle` is acceleration based vehicle. This vehicle model was created for Autoware simulation, and assuming that Autoware has already created a gas pedal map, this vehicle model uses acceleration as an input value. It has the following features:

- Longitudinal control by acceleration ($\frac{m}{s^2}$).
- Lateral control by two-wheel model.
- Yaw, roll and pitch controlled by [*Physics*](https://docs.unity3d.com/Manual/PhysicsSection.html) engine.
- Mass-spring-damper suspension model ([`WheelColliders`](#wheels-colliders)).
- automatic gears change.
- 3D Mesh as road surface for vehicle driving, gradient resistance.

## Prefab

Please refer to `Lexus RX450h 2015.prefab` as a sample prefab

## Related classes

|Class|Feature|
|:--|:--|
|AccelVehicle|The main script of `AccelVehicle`.|
|AccelVehicleGroundSlip|Apply slip to `AccelVehicle`.|
|AccelVehicleVisualEffect|Visuals such as tires and lights for `AccelVehicle`.|
|IReadOnlyAccelVehicle|Read only interface of `AccelVehicle`.|
|AccelVehicleControlModeBasedInputter|Input classes supporting Autoware control input override.|
|AccelVehicleKeyboardInput|Keyboard input for `AccelVehicle`.|
|AccelVehicleLogitechG29Input|Logitech G29 steering wheel input for `AccelVehicle`.|
|AccelVehicleRos2Input|ROS2 input for `AccelVehicle`.|
|IAccelVehicleInput|Interface for input class.|
|AccelVehicleControlModeSrvServer|ROS2 service server with switchable Autoware's vehicle control mode.|
|AccelVehicleReportRos2Publisher|ROS2 publisher to publish topics for Autoware's vehicle report.|
|AccelVehicleRos2MsgConverter|ROS (Autoware) and Unity (Awsim) type conversion.|

## Autoware's vehicle control mode

## Assigned key list
By default, each key is assigned to the following classes.

- AccelVehicleKeyboardInput

|Key|Feature|
|:--|:--|
|D|Switch to move drive gear.|
|R|Switch to move reverse gear.|
|N|Switch to neutral gear.|
|P|Switch to parking gear.|
|Up arrow|Forward acceleration.|
|Down arrow|Reverse acceleration.|
|Left arrow|Left turning.|
|Right arrow|right turning.|
|1|Turn left blinker on.|
|2|Turn right blinker on.|
|3|Turn on hazard lights.|
|4|Turn off blinker or hazard lights.|
|C|Switch control mode `MANUAL` to `AUTONOMOUS`|

- AccelVehicleLogitechG29Input

|Key|Feature|
|:--|:--|
|Triangle|Switch to move drive gear.|
|Square|Switch to move reverse gear.|
|Circle|Switch to neutral gear.|
|Cross|Switch to parking gear.|
|Throttle pedal|Forward acceleration.|
|Brake pedal|Reverse acceleration.|
|Steering wheel|Turning.|
|Left paddle|Turn left blinker on.|
|Right paddle|Turn right blinker on.|
|R2|Turn on hazard lights.|
|R3|Turn off blinker or hazard lights.|
|L2|Switch control mode `AUTONOMOUS` to `MANUAL`|
|L3|Switch control mode `MANUAL` to `AUTONOMOUS`|

## Create custom vehicle

It is easier to understand if you refer to `Lexus RX450h 2015.prefab`.

1. Prepare a 3D model of the vehicle. (Separate the wheels, body and lights)
1. Set up `WheelCollider` for each wheel and `MeshCollider` or other collider for vehicle body.
1. Attach the `Rigidbody` component to the vehicle and configure inspector.
1. Attach the `AccelVehicle.cs` script to the vehicle and configure inspector.
1. Attach the `AccelVehicleVisualEffect.cs` script to the vehicle and configure inspector.
1. Attach the `AccelVehicleControlModeBasedInputter` script the vehicle and configure inspector.
1. Attach and configure any VehicleInput classes (`AccelVehicleKeyboardInput`, `AccelVehicleRos2Input`, `AccelVehicleLogitechG29Input`) that inherits from `IAccelVehicleInput`.
1. (optional) By attaching `AccelVehicleReportRos2Publisher` and `AccelVehicleControlModeSrvServer`, it is possible to connect to Autoware.


## Create custom input

1. Create a new class that inherits from `IAccelVehicleInput`.
1. Implement updates to the values of `IAccelVehicleInput` input values.
1. Reference the custom input class you created to `AccelVehicleControlModeBasedInputter` and use it in either autonomous mode or manual mode.