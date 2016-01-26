# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(velma_sim_gazebo)

find_package(catkin REQUIRED COMPONENTS rtt_ros cmake_modules kdl_parser kuka_lwr_fri barrett_hand_common)

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

find_package(DARTCore 4.3.3 REQUIRED)

find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(orocos_kdl REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

link_directories(
  ${DARTCore_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
  ${orocos_kdl_LIBRARY_DIRS}
  )

include_directories(#include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${DARTCore_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIR}
  )

orocos_use_package( ocl-taskbrowser )
orocos_use_package( ocl-deployment )
orocos_use_package( ocl-logging )
orocos_use_package( orocos-rtt-corba )
orocos_use_package( rtt-typekit )

#set(CMAKE_BUILD_TYPE Debug)

## Default component
orocos_component(velma_gazebo src/velma_gazebo_init.cpp src/velma_gazebo.cpp src/velma_gazebo_orocos.cpp src/velma_gazebo_srdf.cpp)
orocos_component(velma_gazebo_tactile src/velma_gazebo_tactile.cpp)

set_target_properties(velma_gazebo PROPERTIES COMPILE_FLAGS -DRTT_COMPONENT)
target_link_libraries(velma_gazebo
  ${DARTCore_LIBRARIES}
  ${GAZEBO_LIBRARIES} gazebo_physics_dart
  ${catkin_LIBRARIES}
  ${OROCOS-RTT_LIBRARIES} 
  ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} 
  ${OROCOS-RTT_RTT-TRANSPORT-CORBA_LIBRARY}
  ${USE_OROCOS_LIBRARIES}
  ${Boost_LIBRARIES}
)

set_target_properties(velma_gazebo_tactile PROPERTIES COMPILE_FLAGS -DRTT_COMPONENT)
target_link_libraries(velma_gazebo_tactile
  ${DARTCore_LIBRARIES}
  ${GAZEBO_LIBRARIES} gazebo_physics_dart
  ${catkin_LIBRARIES}
  ${OROCOS-RTT_LIBRARIES} 
  ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} 
  ${OROCOS-RTT_RTT-TRANSPORT-CORBA_LIBRARY}
  ${USE_OROCOS_LIBRARIES}
  ${Boost_LIBRARIES}
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
