**VehicleStatusSensor**

(prefab location, purpose of existence, link, **screen**)

- Vehicle Report Ros Publisher Script (inputs, outputs, topics, frame_id, qos)

**RobotecGPULidars [70% current]**

(prefab location, purpose of existence, RGL repository - hyperlink)

- Note: to use RobotecGPULidar, the scene must have Scene Manager Script (hyperlink)
- Lidar Sensor Script
    - Available models (prefabs location, description of differences)
    - Parameters (min/max h angle, max range, horizontal steps)
    - Noise (type and params)
    - Laser array configuration
    - Output (Pcl24 vs Pcl48)
- Rgl Lidar Ros Publisher Script (topics, frame_id, qos)
- Point Cloud Visualization Script (points, colors, limits)

**IMUSensor [50% current]**

(prefab location, purpose of existence, link, **screen**)

- Imu Sensor Script (gravity, output)
- Imu Ros Publisher Script (topics, frame_id, qos)

**GnssSensor [50% current]**

(prefab location, purpose of existence, link, **screen**)

- Gnss Sensor Script (MGRS, output)
- Gnss Ros Publisher Script (topics, frame_id, qos)

**CameraSensor [10% current]**

(prefab location, purpose of existence, link, **screen**)

- Camera component (https://docs.unity3d.com/Manual/class-Camera.html)
- Camera Sensor Script (parameters, output, gui - **screen**)
    - Distortion shader (what is it and where is it located)
    - ROS image shader (what is it and where is it located)
- Camera Ros Publisher Script (topics, frame_id, qos)
- Impact on the traffic lights recognition in autoware
