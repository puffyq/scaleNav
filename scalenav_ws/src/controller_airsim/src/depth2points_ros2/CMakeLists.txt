cmake_minimum_required(VERSION 3.8)
project(depth2points_ros2)

# ROS2's ament tooling must use the system Python, not an active Conda env.
if(EXISTS "/usr/bin/python3")
  set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "ROS2 Python" FORCE)
  set(Python_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "ROS2 Python" FORCE)
  set(PYTHON_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "ROS2 Python" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(depth_planar_to_pointcloud_node
  src/depth_planar_to_pointcloud_node.cpp
)

target_include_directories(depth_planar_to_pointcloud_node PRIVATE include)
ament_target_dependencies(depth_planar_to_pointcloud_node
  rclcpp
  sensor_msgs
)

install(TARGETS depth_planar_to_pointcloud_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES launch/depth_planar_to_pointcloud.launch.py
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_depth_projection test/test_depth_projection.cpp)
  target_include_directories(test_depth_projection PRIVATE include)
endif()

ament_package()
