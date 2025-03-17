# Private Features Showcase

The following list of features is proprietary and only accessible through contact with TIER IV organisation.The presented features extend functionalities of AWSIM by providing many more tools or extending public accessible features.

## Models

Proprietary AWSIM offers more choice of NPCs available in your simulation environments.

The available models can be observed by opening the [Showcase](https://github.com/tier4/AWSIM-mirror/blob/feature/showcase-documentation/Assets/PrivateAssets/PrivateAWSIM/Scenes/Samples/Showcase/Showcase.unity) scene and choosing the `MODELS` from top left dropdown.

![ShowcaseSceneOption.png](Models/ShowcaseSceneOption.png)

The scene showcases all models available in simulator.


### Showcase

All the podels from public AWSIM are available in closed source simulation. It is possible to choose one of the following categories in right pane dropdown and the following models will be shown


<details>
<summary>
NPC Vehicles
</summary>
Small Car </br>
Hatchback</br>
Taxi</br>
Van</br>
Jeeps</br>
Libousines</br>
Police car</br>
Sports cars</br>
SUV</br>
Sport motorbike</br>
</details>
<details>
<summary>
NPC Trucks
</summary>
Ambulance</br>
City bus</br>
School bus</br>
Small bus</br>
Light commercial vehicle</br>
Truck with Gas Trailer</br>
Truck with Platform trailer</br>
</details>
<details>
<summary>
NPC Pedestrians
</summary>
Elegant Man</br>
Casual Man</br>
Biker Men, Women and Children</br>
People with personal belongings (umbrellas, smartphones etc.)</br>
Wheelchair users</br>
</details>
<details>
<summary>
EGO Vehicles
</summary>
Lexus RX 450h 2015</br>
Toyota Taxi</br>
BYD J6</br>
</details>
<details>
<summary>
Miscellaneous
</summary>
Baby stroller</br>
Barricade</br>
Bike stand</br>
Vending machine</br>
Road bump</br>
Constructions</br>
Trash can</br>
Pallet</br>
Safety cones</br>
Traffic Signs</br>
Traffic barriers</br>
Parking meters</br>
Fire hydrants</br>
Kickscooters and bikes</br>
</details>


<video width="640" height="360" controls>
  <source src="Models/PrivateModelsPresentation.mp4" type="video/mp4">
</video>

## Sensors

Private AWSIM extends sensors simulation with possibility to publish LiDAR data using UDP.

### Showcase

The sensors behavior can be observed by opening the [Showcase](https://github.com/tier4/AWSIM-mirror/blob/feature/showcase-documentation/Assets/PrivateAssets/PrivateAWSIM/Scenes/Samples/Showcase/Showcase.unity) scene and choosing the `SENSORS` from top left dropdown.

![ShowcaseSceneOption.png](Sensors/ShowcaseSceneOption.png)

To observe the UDP publishing feature:

- Enable the `Enable UDP Lidar Publishing` on right pane
![LidarUdpEnable](Sensors/LidarUdpEnable.png)
- Download [VeloView](https://www.paraview.org/veloview/) software
- Run and click the sensor option in menu
![VeloViewMenu](Sensors/VeloViewMenu.png)
- Choose `VLP-16` sensor calibration, 2368 LIDAR port and click `OK`
![VeloViewSensorConfig](Sensors/VeloViewSensorConfig.png)
- The UDP LiDAR data is visualized
![VeloViewDataReceived](Sensors/VeloViewDataReceived.png)


## EGO

Private AWSIM extends EGO functionalities with:

- collision with simulation objects detection
- vehicle being stuck detection
- unpredicted lane departure detection

### Showcase

The EGO functionalities can be observed by opening the [Showcase](https://github.com/tier4/AWSIM-mirror/blob/feature/showcase-documentation/Assets/PrivateAssets/PrivateAWSIM/Scenes/Samples/Showcase/Showcase.unity) scene 
and choosing the `EGO` from top left dropdown.

![ShowcaseSceneOption.png](EGO/ShowcaseSceneOption.png)


Ego vehicle contains features which make it easier to determine if the simulation is running successfully. In case of:

- collision detection
- vehicle getting stuck
- vehicle unpredictably leaves it's current lane

a specific information is shown on screen. The siutation is presented below and can be tested in the aforementioned scene.


![EGOCollisions](EGO/EGOCollisions.gif)


## Environments

AWSIM simulates urban environments. In private version the following maps are available:
- Nishinjuku
- Odaiba
- Shiojiri

### Showcase

All available models can be observed by opening the [Showcase](https://github.com/tier4/AWSIM-mirror/blob/feature/showcase-documentation/Assets/PrivateAssets/PrivateAWSIM/Scenes/Samples/Showcase/Showcase.unity) scene and choosing the `ENVIRONMENTS` from top left dropdown.

![ShowcaseSceneOption.png](Environments/ShowcaseSceneOption.png)

The scene showcases all available scenes in the simulator.

<video width="640" height="360" controls>
  <source src="Environments/PrivateEnvironments.mp4" type="video/mp4">
</video>

## Weather Effects

Private AWSIM simulates weather effects which can be adjusted for different simulation scenarios:

- Sunlight position for different latitudes, with respect to sun position at seasons
- rain and snowfall

The effects can be observed by opening the [Showcase](https://github.com/tier4/AWSIM-mirror/blob/feature/showcase-documentation/Assets/PrivateAssets/PrivateAWSIM/Scenes/Samples/Showcase/Showcase.unity) scene and choosing the `WEATHER` from top left dropdown.

![ShowcaseSceneOption.png](Weather/ShowcaseSceneOption.png)

It can be observed that:

- Sun position changes on horizon making different lightning effects
- Right pane contains controls for Sunlight and Camera Precipitation effects

The loaded scene makes it possible to:

- observe sun progressing on the sky
- manually adjust sun position when automatic progress is disabled
- enable precipitation effects
- choose between snow and rainfall
- observe how fall rate affects simulation environment

<video width="640" height="360" controls>
  <source src="Weather/WeatherShowcase.mp4" type="video/mp4">
</video>


## Other Available Features

Besides the presented features, the proprietary AWSIM offers:

- simulation configuration through JSON file, adjustable to client needs
- configurable models choice for simulation run
- synthetic data generation features for AI models usage
