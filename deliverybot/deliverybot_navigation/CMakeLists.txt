cmake_minimum_required(VERSION 3.0.2)
project(deliverybot_navigation)


find_package(catkin REQUIRED COMPONENTS
  deliverybot
  deliverybot_control
  rospy
  tf2
  deliverybot_gazebo
  move_base
)



catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)# add_message_files(


catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
