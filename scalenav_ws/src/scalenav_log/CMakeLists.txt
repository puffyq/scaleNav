cmake_minimum_required(VERSION 3.8)
project(scalenav_log)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(EXISTS "/usr/bin/python3")
  set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "ROS2 Python" FORCE)
  set(Python_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "ROS2 Python" FORCE)
  set(PYTHON_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "ROS2 Python" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(scalenav_log_node src/scalenav_log_node.cpp src/sliding_log_store.cpp)
target_include_directories(scalenav_log_node PRIVATE include)
ament_target_dependencies(scalenav_log_node
  geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs trajectory_msgs visualization_msgs)

add_executable(scalenav_log_viewer src/scalenav_log_viewer.cpp src/sliding_log_store.cpp)
target_include_directories(scalenav_log_viewer PRIVATE include)
ament_target_dependencies(scalenav_log_viewer rclcpp)

install(TARGETS scalenav_log_node scalenav_log_viewer
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY web config launch
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_log_storage test/test_log_storage.cpp src/sliding_log_store.cpp)
  target_include_directories(test_log_storage PRIVATE include)
endif()

ament_package()
