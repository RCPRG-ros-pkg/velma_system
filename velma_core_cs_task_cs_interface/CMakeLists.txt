cmake_minimum_required(VERSION 2.8.3)
project(velma_core_cs_task_cs_interface)

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  diagnostic_msgs
  velma_core_cs_task_cs_msgs
  eigen_conversions
  kdl_conversions cmake_modules
  common_interfaces
  )

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

link_directories(${orocos_kdl_LIBRARY_DIRS})

include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file
orocos_use_package( rtt-typekit )

include_directories(${USE_OROCOS_INCLUDE_DIRS} include)
ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)
### Orocos Targets ###

orocos_component(velma_core_cs_task_cs_interface
    src/velma_core_cs_task_cs_interface.cpp
)

target_link_libraries(velma_core_cs_task_cs_interface
    ${catkin_LIBRARIES}
    ${USE_OROCOS_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${OROCOS-RTT_LIBRARIES})


### Orocos Package Exports and Install Targets ###

# Export package information (replaces catkin_package() macro) 
orocos_generate_package(
  INCLUDE_DIRS include
  LIBRARIES velma_core_cs_task_cs_interface
  CATKIN_DEPENDS rtt_ros geometry_msgs std_msgs message_runtime rtt_roscomm common_interfaces
)

#orocos_install_headers(
#  DIRECTORY include/${PROJECT_NAME}/
#)
