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
- [Prerequisites](./UserGuide/Installation/Prerequisites/) 
- [Run demo](./UserGuide/Installation/RunDemo/) 
- [Set the initialization position](./UserGuide/FirstSteps/SetTheInitializationPosition/)
- [Set a single goal](./UserGuide/FirstSteps/SetASingleGoal/)
- [Add checkpoints on the way to the goal](./UserGuide/FirstSteps/AddCheckpointsOnTheWayToTheGoal/) 
- [Enable self-driving](./UserGuide/FirstSteps/EnableSelf-driving/) 
- [Set a list of goals](./UserGuide/FirstSteps/SetAListOfGoals/) 
  
#### Unity project
- [Prerequisites](./UserGuide/BuildFromSource/Prerequisites/)
- [Unity installation ](./UserGuide/BuildFromSource/UnityInstallation/)
- [Setup Unity Project](./UserGuide/BuildFromSource/SetupUnityProject/)
- [Run scene](./UserGuide/BuildFromSource/RunScene/)

#### Project Guide
- [Introduction](./UserGuide/ProjectGuide/Introduction/)
- [Scene](./UserGuide/ProjectGuide/Components/Scene/)
- [EgoVehicle](./UserGuide/ProjectGuide/Components/EgoVehicle/)
- Sensors: [Vehicle Status](./UserGuide/ProjectGuide/Components/Sensors/VehicleStatus/), [GNSS](./UserGuide/ProjectGuide/Components/Sensors/Gnss/), [LiDAR](./UserGuide/ProjectGuide/Components/Sensors/Lidar/), [IMU](./UserGuide/ProjectGuide/Components/Sensors/Imu/), [Camera](./UserGuide/ProjectGuide/Components/Sensors/Camera/), [GroundTruth Pose](./UserGuide/ProjectGuide/Components/Sensors/GroundTruths/Pose/)
- [Environment](./UserGuide/ProjectGuide/Components/Environment/Environment/)
- [Traffic Components](./UserGuide/ProjectGuide/Components/Environment/TrafficComponents/)
- NPCs: [NPCPedestrian](./UserGuide/ProjectGuide/Components/NPCs/Pedestrian/), [NPCVehicle](./UserGuide/ProjectGuide/Components/NPCs/Vehicle/)
- [LaneletBoundsVisualizer](./UserGuide/ProjectGuide/Components/LaneletBoundsVisualizer/) 
- [RGLUnityPlugin](./UserGuide/ProjectGuide/ExternalLibraries/RGLUnityPlugin/)
- [Ros2Unity](./UserGuide/ProjectGuide/ExternalLibraries/Ros2Unity/)
- [ROS2 topics list](./UserGuide/ProjectGuide/Ros2TopicList/)

#### Tutorials
- New Environment: [Add an Environment](./DeveloperGuide/Tutorials/AddANewEnvironment/AddAnEnvironment/), [Add an TrafficLights](./DeveloperGuide/Tutorials/AddANewEnvironment/AddTrafficLights/), [Load Lanelet2](./DeveloperGuide/Tutorials/AddANewEnvironment/AddARandomTraffic/LoadItemsFromLanelet/), [Add a TrafficIntersection](./DeveloperGuide/Tutorials/AddANewEnvironment/AddARandomTraffic/AddATrafficIntersection/), [Add a RandomTraffic](./DeveloperGuide/Tutorials/AddANewEnvironment/AddARandomTraffic/AddARandomTrafficSimulatorScript/)
- New Vehicle: [Add a Vehicle](./DeveloperGuide/Tutorials/AddANewVehicle/AddAVehicle/), [Add Visual Elements](./DeveloperGuide/Tutorials/AddANewVehicle/AddVisualElements/), [Add Colliders](./DeveloperGuide/Tutorials/AddANewVehicle/AddColliders/), [Add Sensors](./DeveloperGuide/Tutorials/AddANewVehicle/AddSensors/)
- New Scene: [Add a Scene](./DeveloperGuide/Tutorials/AddANewScene/AddAScene/), [Add a SceneManager](./DeveloperGuide/Tutorials/AddANewScene/AddASceneManager/)
- [New Lidar](./DeveloperGuide/Tutorials/AddANewLiDAR/)
- [New custom ROS2 Message](./DeveloperGuide/Tutorials/AddACustomMessage/)
- [Create a PCD](./DeveloperGuide/Tutorials/CreateAPCDFromMesh/)
