cmake_minimum_required(VERSION 2.8.3)
project(velma_core_ve_body)

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  rtt_subsystem
  velma_core_cs_ve_body_msgs
  velma_core_ve_re_lwr_msgs
  eigen_conversions
  common_interfaces
  common_behavior
  controller_common
  lwr_msgs
  geometry_msgs
  velma_ec_gen_msgs
  )

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

find_package(Eigen3 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file
#orocos_use_package( rtt-typekit )

#catkin_package(
#  INCLUDE_DIRS include
#  DEPENDS rtt_ros std_msgs message_runtime rtt_roscomm velma_core_cs_ve_body_msgs
#)

include_directories(${USE_OROCOS_INCLUDE_DIRS} include)
### Orocos Targets ###

ros_generate_rtt_master()

orocos_component(velma_core_ve_body
    src/common_predicates.cpp
    src/predicates.cpp
    src/velma_core_ve_body.cpp
    src/components/bypass.cpp
    src/components/safe.cpp
    src/components/safe_elmo.cpp
    src/components/elmo_driver.cpp
    src/components/safe_lwr.cpp
    src/components/packet_counter.cpp
    src/components/vector_dummy.cpp
    src/components/scalar_zero.cpp
    src/components/hw_state.cpp
    src/components/lwr_status_sync.cpp
    src/components/transparent_cmd.cpp
    src/components/transmissions.cpp
    src/components/ft_sensor.cpp
)
target_link_libraries(velma_core_ve_body ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} rtt-${PROJECT_NAME}-master)

### Orocos Package Exports and Install Targets ###

# Export package information (replaces catkin_package() macro) 
orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros std_msgs message_runtime rtt_roscomm velma_core_cs_ve_body_msgs
)

#orocos_install_headers(
#  DIRECTORY include/${PROJECT_NAME}/
#)
