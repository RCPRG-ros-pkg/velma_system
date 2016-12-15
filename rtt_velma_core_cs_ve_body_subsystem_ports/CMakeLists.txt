cmake_minimum_required(VERSION 2.8.3)
project(rtt_velma_core_cs_ve_body_subsystem_ports)

find_package(catkin REQUIRED COMPONENTS
 rtt_ros
 rtt_subsystem_ports
 common_interfaces
 velma_core_ve_body_re_body_msgs
 velma_core_cs_ve_body_msgs
)
#    rtt_velma_core_ve_body_re_body_msgs
#    rtt_velma_core_ve_body_re_body_subsystem_ports
#    rtt_velma_core_cs_ve_body_msgs

ros_generate_rtt_subsystem_ports(velma_core_cs_ve_body_msgs)

orocos_generate_package(
  DEPENDS velma_core_ve_body_re_body_msgs
  DEPENDS_TARGETS rtt_subsystem_ports
# rtt_velma_core_ve_body_re_body_msgs
# rtt_velma_core_cs_ve_body_msgs
# rtt_std_msgs rtt_geometry_msgs rtt_barrett_hand_controller_msgs rtt_velma_core_ve_body_re_body_subsystem_ports
)
