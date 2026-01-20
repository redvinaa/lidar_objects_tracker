cmake_minimum_required(VERSION 3.8)
project(lidar_objects_tracker)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(service_robot_msgs REQUIRED)

# Find Open3D (handle separately as it's not a ROS package)
find_package(Open3D REQUIRED)


set(DEPS
  ament_cmake
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  geometry_msgs
  tf2
  Eigen3
  sensor_msgs
  visualization_msgs
  angles
  service_robot_msgs
  Open3D)

include_directories(include)

# component
add_library(lidar_objects_tracker SHARED src/lidar_objects_tracker.cpp)
ament_target_dependencies(lidar_objects_tracker ${DEPS})
target_include_directories(lidar_objects_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(export_${PROJECT_NAME})

rclcpp_components_register_node(
  lidar_objects_tracker
  PLUGIN "lidar_objects_tracker::ObjectsTracker"
  EXECUTABLE objects_tracker_node)

install(
  TARGETS
    lidar_objects_tracker
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_lidar_objects_tracker
    test/test_kalman_filter.cpp)
  ament_target_dependencies(test_lidar_objects_tracker ${DEPS})
  target_link_libraries(test_lidar_objects_tracker
    lidar_objects_tracker)
endif()

ament_export_dependencies(${DEPS})
ament_package()
