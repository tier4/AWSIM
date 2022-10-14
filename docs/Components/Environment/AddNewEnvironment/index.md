# Add New Environment

## 1. Create VectorMap

<img src=image_0.png width=700px>
Create a VectorMap using [VectorMapBuilder](https://tools.tier4.jp/feature/vector_map_builder/) from the pcd obtained from the actual LiDAR.


## 2. Create Environment 3D models

<img src=image_1.png width=700px>
Create a 3D model of the environment based on pcd; basically, Unity HDRP performs better when meshes are merged. In addition, occlusion culling and flutter culling cannot be used because the sensor's detection target will disappear. Each traffic light should have a separate GameObject. Also, each light in the traffic light should be split into separate materials.

## 3. Output Pcd from Mesh

It may be difficult to make Enviornment's 3D model as accurate as the actual device. So, create a pcd from mesh in AWSIM. See [PointCloudMapper](../../Environment/PointCloudMapper/index.md)