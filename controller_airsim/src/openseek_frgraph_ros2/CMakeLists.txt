cmake_minimum_required(VERSION 3.8)
project(openseek_frgraph_ros2)

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
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GapCandidate.msg"
  "msg/GapCandidates.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(depth_planar_to_pointcloud_node
  src/depth_planar_to_pointcloud_node.cpp
)

add_executable(frgraph_gap_extractor_node
  src/frgraph_gap_extractor_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/FRGraph/src/planner_manager/src/gap_extractor/gap_extractor.cpp
)

add_executable(frgraph_planner_manager_node
  src/frgraph_planner_manager_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/FRGraph/src/planner_manager/src/gap_extractor/gap_extractor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/FRGraph/src/planner_manager/src/planner_manager.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/FRGraph/src/planner_manager/src/free_regions_graph/free_regions_graph.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/FRGraph/src/planner_manager/src/trajectory_optimization/continuous_violation.cpp
)

target_include_directories(depth_planar_to_pointcloud_node PRIVATE include)
ament_target_dependencies(depth_planar_to_pointcloud_node
  rclcpp
  sensor_msgs
)

target_include_directories(frgraph_gap_extractor_node PRIVATE
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/FRGraph/src/planner_manager/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(frgraph_gap_extractor_node
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
)
target_link_libraries(frgraph_gap_extractor_node ${PCL_LIBRARIES})
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(frgraph_gap_extractor_node "${cpp_typesupport_target}")

target_include_directories(frgraph_planner_manager_node PRIVATE
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/FRGraph/src/planner_manager/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/FRGraph/src/DecompROS/DecompUtil/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/FRGraph/src/third_party/piqp/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(frgraph_planner_manager_node
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
)
target_link_libraries(frgraph_planner_manager_node ${PCL_LIBRARIES})
target_link_libraries(frgraph_planner_manager_node "${cpp_typesupport_target}")

install(TARGETS depth_planar_to_pointcloud_node frgraph_gap_extractor_node frgraph_planner_manager_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_depth_projection test/test_depth_projection.cpp)
  target_include_directories(test_depth_projection PRIVATE include)
endif()

ament_package()
