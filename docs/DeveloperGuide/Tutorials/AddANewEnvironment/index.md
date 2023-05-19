<!-- TODO everything -->
**Create a lanelet2**

- Use VectorMapBuilder and real-world PCD (*.pcd) (**video** -* .osm file creation)

**Create a prefab**

- Guidelines for making 3d models (impact on performance, culling…)
- Add roads, buildings, greenery, signs, road markings… (**screens**)
- Add Traffic Lights
    - Add Traffic Light Script (bulb emission and material configs - **screens**)
- Add a Directional Light (**screen** - example, hyperlink to unity)
- Add a Volume (**screen** - example,, hyperlink to unity)
- Add a Environment Script (MGRS)
- Add NPCPedestrians (guidelines, walker configuration)
- Add a RandomTraffic (hyperlink)

**Create a PCD from Mesh**

- Reasons for making pcd from mesh instead of using real world pcd.
- Usage requirements (osm file, 3d model (prefab))
- Create a scene with 3D model map of the area (**gif**)
- Import osm file (**gif**)
- Add a simple vehicle
    - Add a geometry with collider (**screen**)
    - Add a lidar link (**screen**)
    - Add a Lidar Sensor Script (**screen**)
    - Add a RGL Mapping Adapter Script (explanation, leaf size)
    - Add a Point Cloud Visualization Script
- Add a Scene Manager (hyperlink)
- Add a Point Cloud Mapper Script (description, osm container, world origin, vehicle, output, interval)
- Mapping (description play->stop->result, example - **video**)

## Create a lanelet2

## Create a prefab

### Guidelines for making 3d models

### Add a Environment Script
Add en Environment Script into an Environment Object which should be a child of your Simulation Object.

1. Click on the 'Add Component' button in the Environment object

    ![Add environment script gif](add_environment_script.gif)

1. Search for `Environment` and select it

    ![Search for environment script](search_environment_script.png)

1. Set the MGRS offset position

    - To what is your offset position navigate to your osm file containing the lanelet
    - Open it and find the first `<node>` tag, it should look something like this

        ```xml
        <node id="4" lat="35.68855194431519" lon="139.69142711058254">
            <tag k="mgrs_code" v="54SUE815501"/>
            <tag k="local_x" v="81596.1357"/>
            <tag k="local_y" v="50194.0803"/>
            <tag k="ele" v="34.137"/>
        </node>
        ```

    - From this data copy
        - `local_x` value to Mgrs Offset Position X
        - `local_y` value to Mgrs Offset Position Y
        - `ele` value to Mgrs Offset Position Z
        - All the letters and *only* letters from the beginning of the `mgrs_code` value to Mgrs Grid Zone


### Add roads, buildings, greenery, signs, road markings…

### Add Traffic Lights

#### Add Traffic Light Script

### Add a Directional Light

### Add a Volume

### Add NPCPedestrians

### Add a RandomTraffic

## Create a PCD from Mesh
