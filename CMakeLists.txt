cmake_minimum_required(VERSION 2.8.3)
project(velma_low_level_behaviors)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  velma_low_level_interface_msgs
  common_interfaces
  common_behavior
  )

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file
#orocos_use_package( rtt-typekit )

#catkin_package(
#  INCLUDE_DIRS include
#  DEPENDS rtt_ros std_msgs message_runtime rtt_roscomm behavior_manager_msgs velma_low_level_interface_msgs
#)

include_directories(${USE_OROCOS_INCLUDE_DIRS} include)
### Orocos Targets ###

add_library(velma_low_level_behaviors
    src/behavior_idle.cpp
    src/behavior_safe.cpp
    src/state_idle.cpp
    src/state_safe.cpp
)

target_link_libraries(velma_low_level_behaviors ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Package Exports and Install Targets ###

# Export package information (replaces catkin_package() macro) 
orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros std_msgs message_runtime rtt_roscomm behavior_manager_msgs velma_low_level_interface_msgs
)

#orocos_install_headers(
#  DIRECTORY include/${PROJECT_NAME}/
#)
