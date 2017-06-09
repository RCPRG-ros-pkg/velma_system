# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(velma_sim_gazebo)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
    rtt_ros
    rtt_subsystem_ports
    cmake_modules
    kdl_parser
    lwr_msgs
    barrett_hand_common
    barrett_hand_controller_msgs
    velma_core_ve_re_lwr_msgs
    eigen_conversions
    common_behavior
    common_interfaces
    test_ec_msgs_gen
    shm_comm
)

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

find_package(gazebo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#find_package(DARTCore REQUIRED)
#find_package(FCL REQUIRED)

find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${orocos_kdl_LIBRARY_DIRS}
  )

include_directories(
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIR}
  )

#include_directories(${DARTCore_INCLUDE_DIRS})
#link_directories(${DARTCore_LIBRARY_DIRS})

orocos_use_package( ocl-taskbrowser )
orocos_use_package( ocl-deployment )
orocos_use_package( ocl-logging )
orocos_use_package( orocos-rtt-corba )
orocos_use_package( rtt-typekit )

#set(CMAKE_BUILD_TYPE Debug)

ros_generate_rtt_master()

## Default component
orocos_component(velma_sim_gazebo
    src/lwr_gazebo_init.cpp src/lwr_gazebo.cpp src/lwr_gazebo_orocos.cpp src/manipulator_mass_matrix.cpp
    src/torso_gazebo_init.cpp src/torso_gazebo.cpp src/torso_gazebo_orocos.cpp
    src/barrett_hand_gazebo.cpp src/barrett_hand_gazebo_init.cpp src/barrett_hand_gazebo_orocos.cpp
    src/barrett_tactile_gazebo.cpp
    src/optoforce_gazebo.cpp
    src/ft_sensor_gazebo.cpp src/ft_sensor_gazebo_init.cpp src/ft_sensor_gazebo_orocos.cpp
    src/velma_sim_conversion.cpp
    src/head_gazebo.cpp
    src/velma_sim_library.cpp
)

set_target_properties(velma_sim_gazebo PROPERTIES COMPILE_FLAGS -DRTT_COMPONENT)
target_link_libraries(velma_sim_gazebo
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OROCOS-RTT_LIBRARIES} 
  ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} 
  ${OROCOS-RTT_RTT-TRANSPORT-CORBA_LIBRARY}
  ${USE_OROCOS_LIBRARIES}
  ${Boost_LIBRARIES}
  rtt-${PROJECT_NAME}-master
#  ${DARTCore_LIBRARIES}
#  ${FCL_LIBRARY}
)

orocos_generate_package()

# Install targets
#install(TARGETS
#  lwr_controller
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
  )
