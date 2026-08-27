cmake_minimum_required(VERSION 3.8)
project(scalenav_graph_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS2's ament tooling must use the system Python, not an active Conda env.
if(EXISTS "/usr/bin/python3")
  set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "ROS2 Python" FORCE)
  set(Python_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "ROS2 Python" FORCE)
  set(PYTHON_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "ROS2 Python" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common octree io filters)
find_package(OpenMP REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(TOPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/src/pointcloud_topo)
set(IKD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_map)
set(TOPO_SOURCES
  ${TOPO_ROOT}/src/topo_skeleton_graph.cpp
  ${TOPO_ROOT}/src/historical_odom_graph.cpp
  ${TOPO_ROOT}/src/bubble_split.cpp
  ${TOPO_ROOT}/src/parallel_bubble_astar.cpp
)

add_library(ikd_tree STATIC
  ${IKD_ROOT}/src/ikd_Tree.cpp
)
target_include_directories(ikd_tree PUBLIC
  include
  ${IKD_ROOT}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(ikd_tree PUBLIC PCL_NO_PRECOMPILE)
target_link_libraries(ikd_tree PUBLIC ${PCL_LIBRARIES} pthread)

add_executable(epic_graph_node
  src/scale_manager/src/epic_graph_node.cpp
  ${TOPO_SOURCES}
)

target_include_directories(epic_graph_node PRIVATE
  include
  ${TOPO_ROOT}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(epic_graph_node PRIVATE PCL_NO_PRECOMPILE)
ament_target_dependencies(epic_graph_node
  geometry_msgs nav_msgs pcl_conversions rclcpp sensor_msgs visualization_msgs)
target_link_libraries(epic_graph_node ${PCL_LIBRARIES} OpenMP::OpenMP_CXX ikd_tree)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_gtest(test_route_memory test/test_route_memory.cpp)
  target_include_directories(test_route_memory PRIVATE include ${EIGEN3_INCLUDE_DIRS})

  ament_add_gtest(test_lidar_map test/test_lidar_map.cpp)
  target_include_directories(test_lidar_map PRIVATE
    include
    ${IKD_ROOT}/include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
  target_compile_definitions(test_lidar_map PRIVATE PCL_NO_PRECOMPILE)
  ament_target_dependencies(test_lidar_map rclcpp)
  target_link_libraries(test_lidar_map ${PCL_LIBRARIES} ikd_tree)

  ament_add_gtest(test_topo_semantic
    test/test_topo_semantic.cpp
    ${TOPO_SOURCES}
  )
  target_include_directories(test_topo_semantic PRIVATE
    include
    ${IKD_ROOT}/include
    ${TOPO_ROOT}/include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
  target_compile_definitions(test_topo_semantic PRIVATE PCL_NO_PRECOMPILE)
  ament_target_dependencies(test_topo_semantic
    geometry_msgs nav_msgs pcl_conversions rclcpp sensor_msgs visualization_msgs)
  target_link_libraries(test_topo_semantic ${PCL_LIBRARIES} OpenMP::OpenMP_CXX ikd_tree)

  # End-to-end graph contract: perception cadence, semantic risk and route
  # selection are exercised together with deterministic synthetic inputs.
  ament_add_gtest(test_epic_integration
    test/test_epic_integration.cpp
    ${TOPO_SOURCES}
  )
  target_include_directories(test_epic_integration PRIVATE
    include
    ${IKD_ROOT}/include
    ${TOPO_ROOT}/include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
  target_compile_definitions(test_epic_integration PRIVATE PCL_NO_PRECOMPILE)
  ament_target_dependencies(test_epic_integration
    geometry_msgs nav_msgs pcl_conversions rclcpp sensor_msgs visualization_msgs)
  target_link_libraries(test_epic_integration ${PCL_LIBRARIES} OpenMP::OpenMP_CXX ikd_tree)

  ament_add_gtest(test_rebuild_crash
    test/test_rebuild_crash.cpp
    ${TOPO_SOURCES}
  )
  target_include_directories(test_rebuild_crash PRIVATE
    include
    ${IKD_ROOT}/include
    ${TOPO_ROOT}/include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
  target_compile_definitions(test_rebuild_crash PRIVATE PCL_NO_PRECOMPILE)
  ament_target_dependencies(test_rebuild_crash
    geometry_msgs nav_msgs pcl_conversions rclcpp sensor_msgs visualization_msgs)
  target_link_libraries(test_rebuild_crash ${PCL_LIBRARIES} OpenMP::OpenMP_CXX ikd_tree)

  # Keep the timing simulation in the normal colcon/CTest gate. The report is
  # written into the build tree so a test run never mutates source data.
  add_test(NAME epic_online_simulation
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/epic_online_sim.py
      --json ${CMAKE_CURRENT_BINARY_DIR}/epic_online_sim.json
      --html ${CMAKE_CURRENT_BINARY_DIR}/epic_online_sim_report.html)
endif()

install(TARGETS epic_graph_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
