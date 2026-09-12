cmake_minimum_required(VERSION 3.8)
project(uav_sim)

# ROS2 Humble is installed for the system Python. Avoid an active Conda
# environment selecting an incompatible interpreter during ament configure.
if(EXISTS "/usr/bin/python3")
  set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "ROS2 Python" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(X11 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(uav_sim_node
  src/uav_sim_node.cpp
  src/dynamics/Quadrotor.cpp
)

add_executable(uav_keyboard_node src/uav_keyboard_node.cpp)
target_link_libraries(uav_keyboard_node X11::X11)

target_include_directories(uav_sim_node PRIVATE
  include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(uav_sim_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  trajectory_msgs
)

ament_target_dependencies(uav_keyboard_node
  geometry_msgs
  nav_msgs
  rclcpp
  trajectory_msgs
)

install(TARGETS uav_sim_node uav_keyboard_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
