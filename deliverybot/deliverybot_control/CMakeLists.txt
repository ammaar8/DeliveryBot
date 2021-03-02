cmake_minimum_required(VERSION 3.0.2)
project(deliverybot_control)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    joint_state_controller
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

# catkin_install_python(PROGRAMS
    # scripts/.py
    # DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )