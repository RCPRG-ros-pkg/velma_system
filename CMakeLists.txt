cmake_minimum_required(VERSION 2.8.3)
project(velma_low_level_behaviors)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  velma_low_level_interface_msgs
  velma_low_level_interface
  eigen_conversions
  common_interfaces
  common_behavior
  kuka_lwr_fri
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
#  DEPENDS rtt_ros std_msgs message_runtime rtt_roscomm behavior_manager_msgs velma_low_level_interface_msgs
#)

include_directories(${USE_OROCOS_INCLUDE_DIRS} include)
### Orocos Targets ###

orocos_component(velma_low_level_behaviors
    src/behavior_idle.cpp
    src/behavior_safe.cpp
    src/state_idle.cpp
    src/state_safe.cpp
    src/common_predicates.cpp
    src/velma_low_level_behaviors.cpp
    src/components/diagnostic_component.cpp
    src/components/velma_low_level_master_component.cpp
    src/components/idle_component.cpp
    src/components/safe_component.cpp
)

target_link_libraries(velma_low_level_behaviors ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Package Exports and Install Targets ###

# Export package information (replaces catkin_package() macro) 
orocos_generate_package(
  LIBRARIES velma_low_level_behaviors
  INCLUDE_DIRS include
  DEPENDS rtt_ros std_msgs message_runtime rtt_roscomm behavior_manager_msgs velma_low_level_interface_msgs
)

#orocos_install_headers(
#  DIRECTORY include/${PROJECT_NAME}/
#)
