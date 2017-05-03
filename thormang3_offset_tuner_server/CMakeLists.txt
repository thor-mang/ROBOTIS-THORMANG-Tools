cmake_minimum_required(VERSION 2.8.3)
project(thormang3_offset_tuner_server)

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

catkin_package(
  INCLUDE_DIRS include
)

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
