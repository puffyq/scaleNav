cmake_minimum_required(VERSION 3.8)
project(openseek_epic_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree octree)
find_package(OpenMP REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(OPENSEEK_COMPAT_INCLUDE
  ${CMAKE_CURRENT_SOURCE_DIR}/../openseek_frgraph_ros2/include)
set(EPIC_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/EPIC/src/global_planner)

add_executable(epic_graph_node
  src/epic_graph_node.cpp
  ${EPIC_ROOT}/pointcloud_topo/src/topo_skeleton_graph.cpp
  ${EPIC_ROOT}/pointcloud_topo/src/historical_odom_graph.cpp
  ${EPIC_ROOT}/pointcloud_topo/src/bubble_split.cpp
  ${EPIC_ROOT}/pointcloud_topo/src/parallel_bubble_astar.cpp
)

target_include_directories(epic_graph_node PRIVATE
  include
  ${OPENSEEK_COMPAT_INCLUDE}
  ${EPIC_ROOT}/pointcloud_topo/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(epic_graph_node PRIVATE PCL_NO_PRECOMPILE)
ament_target_dependencies(epic_graph_node
  geometry_msgs nav_msgs pcl_conversions rclcpp sensor_msgs visualization_msgs)
target_link_libraries(epic_graph_node ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_gtest(test_route_memory test/test_route_memory.cpp)
  target_include_directories(test_route_memory PRIVATE include ${EIGEN3_INCLUDE_DIRS})

  ament_add_gtest(test_lidar_map test/test_lidar_map.cpp)
  target_include_directories(test_lidar_map PRIVATE
    include
    ${OPENSEEK_COMPAT_INCLUDE}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
  target_compile_definitions(test_lidar_map PRIVATE PCL_NO_PRECOMPILE)
  ament_target_dependencies(test_lidar_map rclcpp)
  target_link_libraries(test_lidar_map ${PCL_LIBRARIES})

  ament_add_gtest(test_topo_semantic
    test/test_topo_semantic.cpp
    ${EPIC_ROOT}/pointcloud_topo/src/topo_skeleton_graph.cpp
    ${EPIC_ROOT}/pointcloud_topo/src/historical_odom_graph.cpp
    ${EPIC_ROOT}/pointcloud_topo/src/bubble_split.cpp
    ${EPIC_ROOT}/pointcloud_topo/src/parallel_bubble_astar.cpp
  )
  target_include_directories(test_topo_semantic PRIVATE
    include
    ${OPENSEEK_COMPAT_INCLUDE}
    ${EPIC_ROOT}/pointcloud_topo/include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
  target_compile_definitions(test_topo_semantic PRIVATE PCL_NO_PRECOMPILE)
  ament_target_dependencies(test_topo_semantic
    geometry_msgs nav_msgs pcl_conversions rclcpp sensor_msgs visualization_msgs)
  target_link_libraries(test_topo_semantic ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)

  # End-to-end graph contract: perception cadence, semantic risk and route
  # selection are exercised together with deterministic synthetic inputs.
  ament_add_gtest(test_epic_integration
    test/test_epic_integration.cpp
    ${EPIC_ROOT}/pointcloud_topo/src/topo_skeleton_graph.cpp
    ${EPIC_ROOT}/pointcloud_topo/src/historical_odom_graph.cpp
    ${EPIC_ROOT}/pointcloud_topo/src/bubble_split.cpp
    ${EPIC_ROOT}/pointcloud_topo/src/parallel_bubble_astar.cpp
  )
  target_include_directories(test_epic_integration PRIVATE
    include
    ${OPENSEEK_COMPAT_INCLUDE}
    ${EPIC_ROOT}/pointcloud_topo/include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
  target_compile_definitions(test_epic_integration PRIVATE PCL_NO_PRECOMPILE)
  ament_target_dependencies(test_epic_integration
    geometry_msgs nav_msgs pcl_conversions rclcpp sensor_msgs visualization_msgs)
  target_link_libraries(test_epic_integration ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)

  # Keep the timing simulation in the normal colcon/CTest gate. The report is
  # written into the build tree so a test run never mutates source data.
  add_test(NAME epic_online_simulation
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/../../../tools/epic_online_sim.py
      --json ${CMAKE_CURRENT_BINARY_DIR}/epic_online_sim.json
      --html ${CMAKE_CURRENT_BINARY_DIR}/epic_online_sim_report.html)
endif()

install(TARGETS epic_graph_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
