cmake_minimum_required(VERSION 3.8)
project(lidar_objects_tracker_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/TrackedObject.msg
  msg/TrackedObjects.msg
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
)

# Needed so targets can depend on generated messages
ament_export_dependencies(rosidl_default_runtime)

ament_package()
