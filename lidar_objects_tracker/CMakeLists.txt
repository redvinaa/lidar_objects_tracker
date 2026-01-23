cmake_minimum_required(VERSION 3.8)
project(lidar_objects_tracker)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(angles REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(lidar_objects_tracker_msgs REQUIRED)

# Non-ROS dependency
find_package(Open3D REQUIRED)

set(DEPS
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  angles
  Eigen3
  lidar_objects_tracker_msgs
  Open3D
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/lidar_objects_tracker.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  ${DEPS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/lidar_objects_tracker>
  $<INSTALL_INTERFACE:include/lidar_objects_tracker>
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "lidar_objects_tracker::ObjectsTracker"
  EXECUTABLE objects_tracker_node
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${DEPS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_lidar_objects_tracker
    test/test_kalman_filter.cpp
  )

  ament_target_dependencies(test_lidar_objects_tracker ${DEPS})

  target_link_libraries(
    test_lidar_objects_tracker
    ${PROJECT_NAME}
  )
endif()

ament_package()
