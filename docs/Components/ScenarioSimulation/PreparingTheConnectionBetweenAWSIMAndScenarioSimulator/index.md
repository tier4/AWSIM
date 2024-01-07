# Preparing the connection between AWSIM and scenario_simulator_v2

This tutorial describes:
- how to modify scenario to work with AWSIM
- how to prepare the AWSIM scene to work with `scenario_simulator_v2`

## Scenario preparation to work with AWSIM

To prepare the scenario to work with AWSIM add `model3d` field to [entity specification](https://www.asam.net/static_downloads/ASAM_OpenSCENARIO_V1.2.0_Model_Documentation/modelDocumentation/content/Vehicle.html#:~:text=model3d,absolute%20file%20path.)

It is utilized as an asset key to identify the proper prefab.

![model_3d_added.png](model_3d_added.png)

Match the parameters of the configured vehicle to match the entities parameters in AWSIM as close as it is required. Especially the bounding box is crucial to validate the collisions correctly.

### Default AWSIM asset catalog

AWSIM currently supports the following asset key values. 

The list can be extended if required. Appropriate values should be added to asst key list in the `ScenarioSimulatorConnector` component and the vehicle parameters in scenario simulator should match them.

#### Ego Vehicle Entity (with sensor)
| model3d       | boundingbox size (m)                                  | wheel base(m) | front tread(m) | rear tread(m) | tier diameter(m) | max steer(deg) |
|---------------|-------------------------------------------------------|---------------|----------------|---------------|------------------|----------------|
| lexus_rx450h  | width : 1.920 <br> height : 1.700 <br> length : 4.890 | 2.105         | 1.640          | 1.630         | 0.766            | 35             |

#### NPC Vehicle Entity

| model3d   | boundingbox size (m)                                  | wheel base(m) | front tread(m) | rear tread(m) | tier diameter(m) | max steer(deg) |
|-----------|-------------------------------------------------------|---------------|----------------|---------------|------------------|----------------|
| taxi      | width : 1.695 <br> height : 1.515 <br> length : 4.590 | 2.680         | 1.460          | 1.400         |  0.635           | 35             |
| truck_2t  | width : 1.695 <br> height : 1.960 <br> length : 4.685 | 2.490         | 1.395          | 1.240         | 0.673            | 40             |
| hatchback | width : 1.695 <br> height 1.515 <br> length : 3.940   | 2.550         | 1.480          | 1.475         | 0.600            | 35             |
| van       | width : 1.880 <br> height : 2.285 <br> length : 4.695 | 2.570         | 1.655          | 1.650         | 0.600            | 35             |
| small_car | width : 1.475 <br> height 1.800 <br> length : 3.395   | 2.520         | 1.305          | 1.305         | 0.557            | 35             |

#### NPC Pedestrian Entity

| model3d | boundingbox size (m)                                  |
|---------|-------------------------------------------------------|  
| human   | width : 0.400 <br> height : 1.800 <br> length : 0.300 |

#### Misc Object Entity

| model3d    | boundingbox size (m)                               |
|------------|----------------------------------------------------|  
| sign_board | width : 0.31 <br> height : 0.58 <br> length : 0.21 |

### Scenarios limitations

Vast majority of features supported by `scenario_simulator_v2` are supported with AWSIM as well. Currently supported features are described in the [scenario_simulator_v2's documentation](https://tier4.github.io/scenario_simulator_v2-docs/developer_guide/OpenSCENARIOSupport/).

Features which are not supported when connected with AWSIM are listed below.

1. Controller properties used by `attach_*_sensor`
    - `pointcloudPublishingDelay`
    - `isClairvoyant`
    - `detectedObjectPublishingDelay`
    - `detectedObjectPositionStandardDeviation`
    - `detectedObjectMissingProbability`
    - `randomSeed`

If those features are curcial for the scenario's execution, the scenario might not work properly.

## AWSIM scene preparation to work with `scenario_simulator_v2`

1. Disable or remove random traffic and any pre-spawned NPCs
2. Disable or remove V2I traffic lights publishing
3. Disable or remove the clock publisher

  ![removed_objects.png](removed_objects.png)

4. Add `ScenarioSimulatorConnector` prefab to the scene - located in `Assets/ScenarioSimulatorConnector`

  ![scene_tree.png](scene_tree.png)

5. Add Ego Follow Camera object - most likely Main Camera

  ![follow_camera.png](follow_camera.png)

6. If necessary update the asset_id to prefab mapping - key in the map can be used in the scenario 

  ![entities.png](entities.png)

7. Add TimeSourceSelector prefab to the scene - located in `Assets/AWSIM/Scripts/Clock/Prefabs`

  ![scene_tree_time_selector.png](scene_tree_time_selector.png)

8. Configure Type in the TimeSourceSelector component to SS2

  ![time_selector_ss2.png](time_selector_ss2.png)



