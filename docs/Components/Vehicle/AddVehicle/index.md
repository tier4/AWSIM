# Add New Vehicle

This document describes process of adding new vehicle model to Unity project.

## Prepare Vehicle 3D model

![](image_0.png)

A good example of how to prepare the 3D vehicle model prefab is Lexus RX450h which is already added and used in AWSIM project.
The model can be found under `Assets/AWSIM/Models/Vehicles/Lexus RX450h 2015` directory.

To add new model user have to divide the model's meshes in the following parts:

- Body
- Each wheel
- Steering

The division needs also to be applied regarding the materials:

- HeadLight
- BrakeLight
- TurnSignal
- ReverseLight

## Setup Physics & Collider

![](image_1.png)

To make the vehicle model properly interact with other parts of simulation system, the following steps need to be conducted:

1. Attach and configure `Rigidbody`
2. Create `MeshCollider` for models body
3. Set up `WheelCollider` on each wheel. Also attach the editor extension script for `WheelColliderConfig.cs`, which automatically sets unnecessary parameters of `WheelCollider`. Set up parameters through the `WheelColliderConfig` inspector.
## Attach control scripts

The last step is to attach proper component scripts to the vehicle model prefab. The scripts will make the vehicle controllable and interactive with Autoware:

1. Wheel.cs

    Attach `Wheel.cs` script to each `WheelCollider` in the 3D model

    |property|feature|
    |:--|:--|
    |WheelCollider|See [Unity documentation](https://docs.unity3d.com/Manual/class-WheelCollider.html)|
    |WheelVisualTransform|Reference to WheelVisual Object|


2. Vehicle.cs

    Attach `Vehicle.cs` script to vehicle root GameObject

    |property|feature|
    |:--|:--|
    |CenterOfMass|Center of Mass position set by transform|
    |UseInertia|Flag enabling moment of inertia usage|
    |Inertia|Moment of inertia when `UseInertia` is enabled (kgm^2)|
    |Front, Rear Axle|Reference to each `Wheel` component|

3. VehicleVisualEffect.cs

    Attach `VehicleVisualEffect.cs` script to the GameObject to which `Vehicle.cs` is attached

    |property|feature|
    |:--|:--|
    |Vehicle|Reference to `Vehicle` component|
    |Brake Lights|Brake light materials|
    |Left, Right Turn Signal Lights|Turn signal light materials|
    |Turn Signal Timer Interval Sec|Blink time of turn signal light|
    |Reverse Lights|Reverse light materials|


4. VehicleROSInput.cs

    Attach `VehicleROSInput.cs` script to the GameObject to which `Vehicle.cs` is attached. You can configure the topic of each msg to subscribe in ROS2. When this script is attached, default values are set.

!!! info
    Attach `VehicleKeyboardInput.cs` for keyboard operation instead.
