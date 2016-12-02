cmake_minimum_required(VERSION 2.8.3)
project(rtt_velma_core_cs_task_cs_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm common_interfaces velma_core_cs_task_cs_msgs)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIR})

ros_generate_rtt_typekit(velma_core_cs_task_cs_msgs)

orocos_generate_package(
  DEPENDS velma_core_cs_task_cs_msgs
  DEPENDS_TARGETS rtt_roscomm rtt_std_msgs rtt_geometry_msgs rtt_cartesian_trajectory_msgs

)
