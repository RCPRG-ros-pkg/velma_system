cmake_minimum_required(VERSION 2.8.3)
project(rtt_velma_ec_gen_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm velma_ec_gen_msgs common_interfaces)

ros_generate_rtt_typekit(velma_ec_gen_msgs)

orocos_generate_package(
  DEPENDS velma_ec_gen_msgs
  DEPENDS_TARGETS rtt_roscomm rtt_std_msgs rtt_geometry_msgs
)
