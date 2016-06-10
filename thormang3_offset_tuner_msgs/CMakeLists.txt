cmake_minimum_required(VERSION 2.8.3)
project(thormang3_offset_tuner_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
   FILES
   JointOffsetData.msg
   JointOffsetPositionData.msg
   JointTorqueOnOff.msg
   JointTorqueOnOffArray.msg
)

add_service_files(
   FILES
   GetPresentJointOffsetData.srv
)

 generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

