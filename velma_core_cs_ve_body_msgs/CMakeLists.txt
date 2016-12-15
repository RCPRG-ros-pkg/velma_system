cmake_minimum_required(VERSION 2.8.6)

project(velma_core_cs_ve_body_msgs)

find_package(catkin REQUIRED
  COMPONENTS std_msgs geometry_msgs barrett_hand_controller_msgs common_interfaces message_generation actionlib_msgs velma_core_ve_body_re_body_msgs)

add_message_files(FILES
#  CommandArm.msg
#  CommandHand.msg
#  CommandSimple.msg
#  CommandMotor.msg
  Command.msg
#  StatusArm.msg
#  StatusHand.msg
#  StatusFT.msg
#  StatusMotor.msg
  StatusSC.msg
  Status.msg
  )
  

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  barrett_hand_controller_msgs
  velma_core_ve_body_re_body_msgs
  )


catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs barrett_hand_controller_msgs message_runtime velma_core_ve_body_re_body_msgs)
