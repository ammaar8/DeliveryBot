cmake_minimum_required(VERSION 3.0.2)
project(deliverybot_navigation)


find_package(catkin REQUIRED COMPONENTS
  deliverybot
  deliverybot_control
  rospy
  tf2
  deliverybot_gazebo
  move_base
  std_msgs
  message_generation
)

add_service_files(
  FILES
  MapFilePath.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
  scripts/navigation_map_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
