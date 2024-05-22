cmake_minimum_required(VERSION 3.5)
project(entity_controller_msgs)


find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/SpawnEntity.msg"
  "msg/UpdatePoseEntity.msg"
  "msg/DespawnEntity.msg"
)


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_package()
