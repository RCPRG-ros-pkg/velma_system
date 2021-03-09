cmake_minimum_required(VERSION 2.8.3)
project(velma_look_at_action_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm geometry_msgs sensor_msgs message_generation actionlib_msgs)

add_action_files(DIRECTORY action FILES LookAt.action)

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs actionlib_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs sensor_msgs message_runtime)
