cmake_minimum_required(VERSION 2.8.3)
project(velma_common_components)

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  controller_common
  )

include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

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


orocos_service(ec_can_queue
    src/ec_can_queue.cpp
)
target_link_libraries(ec_can_queue ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

orocos_component(velma_common_components
    src/ec_can_tx_queue_component.cpp
)
target_link_libraries(velma_common_components ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Package Exports and Install Targets ###

# Export package information (replaces catkin_package() macro) 
orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros message_runtime rtt_roscomm
  DEPENDS_TARGETS controller_common
)

#orocos_install_headers(
#  DIRECTORY include/${PROJECT_NAME}/
#)
