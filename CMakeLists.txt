cmake_minimum_required(VERSION 2.8.3)
project(util_3dnav)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)

find_package(unitree_sdk2 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(unitree_cmd_vel_bridge
  src/unitree_cmd_vel_bridge.cpp
)

target_link_libraries(unitree_cmd_vel_bridge
  ${catkin_LIBRARIES}
  unitree_sdk2
)

catkin_install_python(PROGRAMS
  scripts/midware_bridge_node.py
  scripts/simulation_cmd_vel_bridge.py
  scripts/simulation_waypoints_to_nodes_json.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
