cmake_minimum_required(VERSION 2.8.3)
project(rtt_velma_core_ve_re_lwr_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm velma_core_ve_re_lwr_msgs common_interfaces)

ros_generate_rtt_typekit(velma_core_ve_re_lwr_msgs)

orocos_generate_package(
  DEPENDS velma_core_ve_re_lwr_msgs
  DEPENDS_TARGETS rtt_roscomm rtt_lwr_msgs rtt_std_msgs
)
