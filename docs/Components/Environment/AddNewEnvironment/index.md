# Add new environment

The document describes how to add custom, simulated environment to AWSIM project.

## 1. Create Lanelet2

Create a Lanelet2 using [VectorMapBuilder](https://tools.tier4.jp/feature/vector_map_builder/) from the PCD obtained from real-life LiDAR sensor.

<img src=image_0.png width=700px>

## 2. Create 3D models

<img src=image_1.png width=700px>

To properly create 3D models of the environment please keep in mind the following notes:

- 3D models creation can be done based on PCD data, using [VectorMapBuilder](https://tools.tier4.jp/feature/vector_map_builder/).
- AWSIM is created using HDRP (High Definition Rendering Pipeline) which performs better when object meshes are merged.
- Occlusion culling and flutter culling cannot be used because the sensors detection target will disappear.
- Each traffic light should have a separate GameObject. Also, each light in the traffic light should be split into separate materials.

## 3. Output PCD from Mesh

It may not be easy to make Environment's 3D model as accurate as from the real sensor. However, it is possible to create a PCD map from object meshes in AWSIM. Please refer to [PointCloudMapper](../../Environment/PointCloudMapper/index.md) document to see how to create a PCD map using a previously existent digital twin of the environment.
