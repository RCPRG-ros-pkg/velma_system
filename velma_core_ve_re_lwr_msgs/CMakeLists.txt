cmake_minimum_required(VERSION 2.8.6)

project(velma_core_ve_re_lwr_msgs)

find_package(catkin REQUIRED
  COMPONENTS message_generation lwr_msgs std_msgs)

add_message_files(FILES
  Command.msg
  Status.msg
  )
  

generate_messages(
    DEPENDENCIES
    lwr_msgs
    std_msgs)


catkin_package(
  CATKIN_DEPENDS message_runtime lwr_msgs std_msgs)
