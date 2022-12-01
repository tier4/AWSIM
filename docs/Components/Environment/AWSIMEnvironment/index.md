# AWSIM Environment

The following document describes the environment simulated in AWSIM.

## Required Environment files

AWSIM and Autoware requires the following three Environment files to work properly.

|file|use|purpose|
|:--|:--|:--|
|lanelet2 (.osm)|Autoware|Lane information defining traffic rules.|
|pointcloud (.pcd)|Autoware|Point cloud information used for Autoware's localization. |
|3D model (.fbx)|AWSIM|3D model of the simulated environment within AWSIM.|

## Overview

AWSIM's Environment contains the following elements.

- Buildings, Roads
- Traffic lights
- NPC traffic
    - Vehicle
    - Pedestrian

## Tokyo West Shinjuku, Japan
<img src=image_0.png width=600px>

The sample 3D map of Tokyo West Shinjuku, Japan set up as a sample is available for distribution.
See [SetupUnityProject](../../../GettingStarted/SetupUnityProject/#5-import-environment-custom-package), where you can download the unitypackage.

### Buildings and roads

Buildings have the greatest impact on Autoware's self-position estimation. Accurate environment object placement is crucial for correct point cloud data generation, which directly affects localization capabilities.
Additionally, the road surface affects vehicle dynamics.

### Traffic lights control

To simulate signal recognition, the modeled traffic lights must be strictly aligned with the Lanelet2 location.
NPCVehicle runs based on traffic light control, which defines traffic flow at intersections, preventing the NPCs from colliding.
Additionally, ego vehicle performs signal recognition from camera sensor images containing traffic lights.

## NPC traffic

The use of NPCs adds realism to self-driving simulations, especially in urban environments. NPC Vehicle, NPC Pedestrian to simulate traffic.
