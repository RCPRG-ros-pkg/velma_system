cmake_minimum_required(VERSION 2.8.3)
project(rtt_velma_ec_gen_subsystem_buffers)

find_package(catkin REQUIRED COMPONENTS
 rtt_ros
 rtt_subsystem_buffers
 common_interfaces
 velma_ec_gen_msgs
)

ros_generate_rtt_subsystem_buffers(ENABLE_CONVERTER PACKAGE velma_ec_gen_msgs MESSAGES EcInput EcOutput)

orocos_generate_package(
  DEPENDS velma_ec_gen_msgs
  DEPENDS_TARGETS rtt_subsystem_buffers)
