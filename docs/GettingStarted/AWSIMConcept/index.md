
<popup-img src="image_1.png" alt="image_1"></popup-img>
[*AWSIM*](https://github.com/tier4/AWSIM) is an open-source simulator made with [*Unity*](https://unity.com/) for autonomous driving research and development. It is developed for self-driving software like [*Autoware*](../Autoware/). This simulator aims to bridge the gap between the virtual and real worlds, enabling users to train and evaluate their autonomous systems in a safe and controlled environment before deploying them on real vehicles. It provides a realistic virtual environment for training, testing, and evaluating various aspects of autonomous driving systems.

AWSIM simulates a variety of real-world scenarios, with accurate physics and sensor models. It offers a wide range of sensors, such as: Cameras, GNSS, IMU and LiDARs, allowing developers to simulate their autonomous vehicle's interactions with the environment accurately. The simulator also models dynamic objects, such as pedestrians, other vehicles, and traffic lights, making it possible to study interactions and decision-making in complex traffic scenarios. This enables the testing and evaluation of perception, planning, and control algorithms under different sensor configurations and scenarios.

## AWSIM features
- Optimally integrated simulation with Autoware
- Distribution of demo simulations highly optimized for Autoware
- ROS2 native communications and environments
- Use the same ROS2 topics and messages as the actual vehicle
- Vehicle dynamics optimized for Autoware
- Support for Autoware's vehicle control mode
- Support Logitech G29 steering wheels
- Ray tracing lidar sensor simulation
- Open CV camera sensor simulation
- IMU, GNSS sensor simulation
- Controllable time scale
- Support V2I development
- Random traffic simulation. Seed values can also be fixed
- point cloud generation
- Switchable between HDRP and URP
- Focus on customizability through coding
- Support ASAM OpenSCENARIO (connecting scenario simulator v2)
- Simple and highly expandable layered architecture
- Controlling the execution order of C# code independent of Unity's script execution order