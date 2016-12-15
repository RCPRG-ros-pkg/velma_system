cmake_minimum_required(VERSION 2.8.3)
project(rtt_velma_core_cs_task_cs_subsystem_ports)

find_package(catkin REQUIRED COMPONENTS
 rtt_ros
 rtt_subsystem_ports
 common_interfaces
 velma_core_cs_task_cs_msgs
)

ros_generate_rtt_subsystem_ports(velma_core_cs_task_cs_msgs)

orocos_generate_package(
  DEPENDS velma_core_ve_body_re_body_msgs
  DEPENDS_TARGETS rtt_subsystem_ports)
