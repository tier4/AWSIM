# Create pcd

## Demo scene

Demo scene path: `Assets\Awsim\Scenes\PcdGenerationSample.unity`

## Create pcd with custom map

1. Prepare 3d model and lanelet(.osm) file.
1. Create new scene and place the map.
1. Copy the entire contents of the `PcdGenerationSample.unity` scene to a new scene.
1. Configure PcdGenerator. (GameObject path: `PcdGenerationSample\PcdGenerator`)
    1. Set osm of custom map to `OsmDataContainer` in PcdGenerator.
    1. Set the ROS coordinates of the custom map origin to `WorldOriginROS` in PcdGenerator.
1. Run and wait point cloud mapping.