cmake_minimum_required(VERSION 2.8.6)

project(velma_low_level_interface_msgs)

find_package(catkin REQUIRED
  COMPONENTS std_msgs geometry_msgs barrett_hand_controller_msgs message_generation actionlib_msgs)

add_message_files(FILES
  VelmaLowLevelCommandArm.msg
  VelmaLowLevelCommandHand.msg
  VelmaLowLevelCommandSimple.msg
  VelmaLowLevelCommand.msg
  VelmaLowLevelStatusArm.msg
  VelmaLowLevelStatusHand.msg
  VelmaLowLevelStatusFT.msg
  VelmaLowLevelStatusMotor.msg
  VelmaLowLevelStatus.msg
  )
  

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  barrett_hand_controller_msgs
  )


catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs barrett_hand_controller_msgs message_runtime)
