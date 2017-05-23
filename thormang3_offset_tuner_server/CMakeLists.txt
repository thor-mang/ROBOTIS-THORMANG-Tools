################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(thormang3_offset_tuner_server)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  cmake_modules
  dynamixel_sdk
  robotis_device
  robotis_math
  robotis_framework_common
  robotis_controller
  robotis_controller_msgs
  thormang3_kinematics_dynamics
  thormang3_base_module
  thormang3_offset_tuner_msgs
)

find_package(Eigen REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(thormang3_offset_tuner_server_node 
   src/thormang3_offset_tuner_server.cpp
   src/main.cpp
)

add_dependencies(thormang3_offset_tuner_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(thormang3_offset_tuner_server_node
  yaml-cpp
  ${catkin_LIBRARIES}
)

################################################################################
# Install
################################################################################
install(TARGETS thormang3_offset_tuner_server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

################################################################################
# Test
################################################################################
