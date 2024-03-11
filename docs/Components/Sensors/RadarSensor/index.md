# RadarSensor

## Introduction

The `RadarSensor` component simulates a radar sensor that detect objects in the environment.
Real-world radar sensors work by emitting radio waves and detecting the waves that are reflected back from objects in
the environment.

`RadarSensor` implements simplified a model of wave propagation and reflection using GPU-accelerated ray casting and
post-processing
to obtain radar-specific information such as radial (aka doppler)
speed, [RCS](https://en.wikipedia.org/wiki/Radar_cross_section),
power level, noise level and signal-to-noise ratio.

`RadarSensor` component is a part of [`RGLUnityPlugin`](../LiDARSensor/RGLUnityPlugin) that integrates the external [
*RobotecGPULidar*](https://github.com/RobotecAI/RobotecGPULidar) (`RGL`) library with *Unity*.

!!! warning "Use RGL in your scene"
If you want to use `RGL` in your scene, make sure the scene has
an [`SceneManager` component](../LiDARSensor/RGLUnityPlugin/#scenemanager) added and all objects meet
the [usage requirements](../LiDARSensor/RGLUnityPlugin/#usage-requirements).

## Example

On the screenshot below (scene `RadarSceneDevelop`) Radar detections are shown as blue boxes.

![RadarTestScene.png](radar_test_scene.png)

## Prefab

Prefabs can be found under the following path:

Assets/AWSIM/Prefabs/RobotecGPULidars/*

| LiDAR                  | Path                          | Appearance                                            |
|:-----------------------|:------------------------------|:------------------------------------------------------|
| *SmartmicroDRVEGRD169* | `SmartmicroDRVEGRD169.prefab` | <img src=prefab_smartmicroDRVEGRD169.png width=150px> |

## Components

The `RadarSensor` component is shown and explained below:

![radar_component.png](radar_component.png)

#### Scope Parameters

Radar Sensor ability to separate (distinguish) different detections varies with the distance.
Scope parameters allow to configure separation thresholds for different distance ranges (scopes).  

  - **Begin Distance** - begin of the distance interval where the following parameters are used.
  - **End Distance** - end of the distance interval where the following parameters are used.
  - **Distance Separation Threshold** - the minimum distance between two points to be considered as separate detections.
  - **Radial Speed Seperation Threshold** - the minimum radial speed difference between two points to be considered as separate detections.
  - **Azimuth Separation Threshold** - the minimum azimuth difference between two points to be considered as separate detections.

