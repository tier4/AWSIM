# AWSIM Environment

## Required Environment files
AWSIM and Autowade require the following three Environment files.

|file|use|purpose|
|:--|:--|:--|
|lanelet2 (.osm)|Autoware|Autoware uses lane information.|
|pointcloud (.pcd)|Autoware|Autoware uses point cloud information. |
|3D model (.fbx)|AWSIM|Used to render the environment within AWSIM.|

## Overview

AWSIM's Environment includes the following.

- Building, Road
- Traffic lights
- Traffic NPC
    - Vehicle
    - Pedestrian

## Tokyo West Shinjuku, Japan
<img src=image_0.png width=600px>

The sample 3D map of Tokyo West Shinjuku, Japan set up as a sample is available for distribution.
See [SetupUnityProject](../../../GettingStarted/SetupUnityProject/#5-import-environment-custom-package), where you can download the unitypackage.

## Building, Road
Building has the greatest impact on self-position estimation by LiDARSensor. The road surface affects vehicle dynamics.

## Traffic lights
To simulate signal recognition, the Traffic light must be strictly aligned with the VectorMap location. NPCVehicle runs based on traffic light control. Ego Vehicle performs signal recognition from camera sensor images.

## Traffic NPC
The use of NPCs can add realism to self-driving simulations. NPCVehicle, NPCPedestrian to simulate traffic.
