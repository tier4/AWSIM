# Smoke Simulator
<img src=image_0.png>
Simulating smoke in AWSIM may be useful when one wants to simulate exhaust gases from vehicles, smoke from emergency flare, etc.

In Unity, it is common to use Particle System to simulate smokes.
However, smoke simulated by Particle System cannot be sensed by RGL in AWSIM although in reality, smokes are detected by LiDAR.

`Smoke Simulator` was developed to simulate smokes that can be detected by RGL in Unity.
`Smoke Simulator` works by instantiating many small cubic GameObjects called `Smoke Particles` and allows each particle to be detected by RGL.

This document describes how to use the `Smoke Simulator`.

## Setting Smoke Simulator

1.Create an empty GameObject

2.Attach `SmokeGenerator.cs` to the previously created GameObject.

<img src=image_1.png>

3.Adjust the parameters of the `SmokeGenerator` as described below:

- `Max Particle`: Specifies the maximum number of particles created by the `Smoke Generator`
- `Particle Range Radius`: Specifies the radius of a circle, centered at the GameObject, which defines the region in which `Smoke Particles` are generated in
- `Particle Size`: Specifies the edge of a `Smoke Particle`
- `Average Lifetime`: Specifies the average lifetime of a `Smoke Particle`
- `Variation Lifetime`: Specifies the variation of lifetime of `Smoke Particles`.

    The lifetime of a `Smoke Particle` is calculated as follows:

        `lifetime` = `Average Lifetime` + Random.Range(-`Variation Lifetime`, `Variation Lifetime`)

- `Physics`: These parameters can be adjusted to specify the behavior of the smoke particles.
    - `Initial Plane Velocity`: Specifies the velocity of a `SmokeParticle` in the x-z plane
    - `Initial Vertical Velocity`: Specifies the velocity of a `SmokeParticle` in the vertical direction
    - `Plane Acceleration`: Specifies the acceleration of a `SmokeParticle` in the x-z plane
    - `Vertical Acceleration`: Specifies the acceleration of a `SmokeParticle` in the vertical direction


4.(Optional): You may also specify the `Material` of `Smoke Particles`. If this field is unspecified, a default material is used

<img src=image_2.png>

## Example of Different Smokes
#### Thin Smoke
<img src=image_3.png>

#### Heavy Smoke
<img src=image_4.png>

