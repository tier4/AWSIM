# Create a PCD from Mesh

!!! Draft-note
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

    Remember to add pcd downsampling:<br>
    `pcl_voxel_grid output_mgrs_local.pcd output_leaf_0_2_mgrs_local.pcd -leaf 0.2 0.2 0.2`
    `pcl_convert_pcd_ascii_binary output_leaf_0_2_mgrs_local.pcd output_ascii_mgrs_local.pcd 0`
