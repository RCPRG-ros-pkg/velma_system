cmake_minimum_required(VERSION 2.8.6)

project(velma_core_cs_ve_body_msgs)

find_package(catkin REQUIRED
  COMPONENTS std_msgs geometry_msgs barrett_hand_controller_msgs message_generation actionlib_msgs)

add_message_files(FILES
  VelmaLowLevelCommandArm.msg
  VelmaLowLevelCommandHand.msg
  VelmaLowLevelCommandSimple.msg
  VelmaLowLevelCommandMotor.msg
  VelmaLowLevelCommand.msg
  VelmaLowLevelStatusArm.msg
  VelmaLowLevelStatusHand.msg
  VelmaLowLevelStatusFT.msg
  VelmaLowLevelStatusMotor.msg
  VelmaLowLevelStatusSC.msg
  VelmaLowLevelStatus.msg
  VelmaRealEffectorStatus.msg
  VelmaRealEffectorCommand.msg
  VelmaRealEffectorArmFriIntf.msg
  VelmaRealEffectorArmFriRobot.msg
  )
  

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  barrett_hand_controller_msgs
  )


catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs barrett_hand_controller_msgs message_runtime)
