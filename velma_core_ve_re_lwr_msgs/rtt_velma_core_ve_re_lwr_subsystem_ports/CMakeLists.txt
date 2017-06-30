cmake_minimum_required(VERSION 2.8.3)
project(rtt_velma_core_ve_re_lwr_subsystem_ports)

find_package(catkin REQUIRED COMPONENTS
 rtt_ros
 rtt_subsystem_ports
 common_interfaces
 velma_core_ve_re_lwr_msgs
)

ros_generate_rtt_subsystem_ports(PACKAGE velma_core_ve_re_lwr_msgs MESSAGES Command Status)

orocos_generate_package(
  DEPENDS velma_core_ve_re_lwr_msgs
  DEPENDS_TARGETS rtt_subsystem_ports
)
