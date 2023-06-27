# Welcome to AWSIM
<video width="1920" controls autoplay muted loop>
<source src="Introduction/AWSIM/awsim_video.mp4" type="video/mp4">
</video>

*AWSIM* is the best scene simulator for [*Autoware*](https://github.com/autowarefoundation/autoware).

Details about what exactly *AWSIM* is and why it was developed can be found [here](./Introduction/AWSIM/).<br> 
You can find out what *Autoware* is by reading this [section](./Introduction/Autoware/), and how it works with *AWSIM* in this [section](./Introduction/ConnectionWithAutoware/).

In addition, *AWSIM* has been adapted to work with *Scenario Simulator v2* - *Autoware* testing framework - you can read about it [here](./Introduction/ConnectionWithScenarioSimulator/).

### AWSIM Features 

- Many predefined components included (Vehicle dynamic models, Sensor models, Environment configuration, *ROS2* communication, etc.)
- Support for *Ubuntu 22.04* and* Windows 10/11*
- *ROS2* native communication (*Humble*)
- [Open sourced](https://github.com/tier4/AWSIM)
- Made with [*Unity*](https://unity.com/)

### Try the simulation demo yourself!
[Download AWSIM Demo for Ubuntu](https://github.com/tier4/AWSIM/releases/download/v1.1.0/AWSIM_v1.1.0.zip){.md-button .md-button--primary}

### Document Guide
If you would just like to run *AWSIM* for the first time, please go to [Prerequisites](./UserGuide/Installation/Prerequisites/) then [Run demo](./UserGuide/Installation/RunDemo/).<br>
If you have encountered any problems please go to section [Troubleshooting](./DeveloperGuide/TroubleShooting/).<br>
Other quick links to the most important parts of the documentation are listed below.

#### Demo
- Prerequisites 
- Run demo
- Set the initialization position 
- Set a single goal 
- Add checkpoints on the way to the goal 
- Enable self-driving 
- Set a list of goals 
  
#### Unity project
- Prerequisites 
- Unity installation 
- Setup Unity Project
- Run scene 

#### Project Guide
- Introduction
- EgoVehicle
- Sensors: Vehicle Status, GNSS, LiDAR, IMU, Camera, GroundTruth Pose
- Environment
- Traffic Components
- NPCPedestrian
- NPCVehicle
- Scene
- RGLUnityPlugin
- Ros2Unity
- ROS2 topics list
- LaneletBoundsVisualizer 

#### Tutorials
- New Environment: Add an Environment, Add an TrafficLights, Add a RandomTraffic
- New Vehicle: Add a Vehicle, Add Visual Elements, Add Colliders, Add Sensors
- New Scene: Add a Scene, Add a SceneManager, Test a Scene
- New Lidar
- New custom ROS2 Message
- Create a PCD
