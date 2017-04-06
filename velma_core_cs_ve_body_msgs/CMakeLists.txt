cmake_minimum_required(VERSION 2.8.6)

project(velma_core_cs_ve_body_msgs)

find_package(catkin REQUIRED
  COMPONENTS std_msgs geometry_msgs barrett_hand_controller_msgs common_interfaces message_generation actionlib_msgs)

add_message_files(FILES
  Command.msg
  CommandArm.msg
  CommandHand.msg
  CommandMotor.msg
  Status.msg
  StatusArm.msg
  StatusFT.msg
  StatusHand.msg
  StatusMotor.msg
  StatusSC.msg
  )
  

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  barrett_hand_controller_msgs
  )


catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs barrett_hand_controller_msgs message_runtime)
