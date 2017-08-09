cmake_minimum_required(VERSION 2.8.3)
project(rtt_velma_core_cs_task_cs_subsystem_buffers)

find_package(catkin REQUIRED COMPONENTS
 rtt_ros
 rtt_subsystem_buffers
 common_interfaces
 velma_core_cs_task_cs_msgs
)

ros_generate_rtt_subsystem_buffers(PACKAGE velma_core_cs_task_cs_msgs MESSAGES Command Status)

orocos_generate_package(
  DEPENDS velma_core_cs_ve_body_msgs
  DEPENDS_TARGETS rtt_subsystem_buffers)
