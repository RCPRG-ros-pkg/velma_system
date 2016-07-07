# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(velma_sim_gazebo)

find_package(catkin REQUIRED COMPONENTS rtt_ros cmake_modules kdl_parser kuka_lwr_fri barrett_hand_common barrett_hand_controller_msgs)

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

find_package(gazebo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

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
#  /usr/include/gazebo-7/gazebo
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
orocos_component(lwr_gazebo src/lwr_gazebo_init.cpp src/lwr_gazebo.cpp src/lwr_gazebo_orocos.cpp src/manipulator_mass_matrix.cpp)
orocos_component(torso_gazebo src/torso_gazebo_init.cpp src/torso_gazebo.cpp src/torso_gazebo_orocos.cpp)
orocos_component(barrett_hand_gazebo src/barrett_hand_gazebo.cpp src/barrett_hand_gazebo_init.cpp src/barrett_hand_gazebo_orocos.cpp)
orocos_component(barrett_tactile_gazebo src/barrett_tactile_gazebo.cpp)
orocos_component(optoforce_gazebo src/optoforce_gazebo.cpp)
orocos_component(ft_sensor_gazebo src/ft_sensor_gazebo.cpp src/ft_sensor_gazebo_init.cpp src/ft_sensor_gazebo_orocos.cpp)

set_target_properties(lwr_gazebo PROPERTIES COMPILE_FLAGS -DRTT_COMPONENT)
target_link_libraries(lwr_gazebo
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OROCOS-RTT_LIBRARIES} 
  ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} 
  ${OROCOS-RTT_RTT-TRANSPORT-CORBA_LIBRARY}
  ${USE_OROCOS_LIBRARIES}
  ${Boost_LIBRARIES}
)

set_target_properties(torso_gazebo PROPERTIES COMPILE_FLAGS -DRTT_COMPONENT)
target_link_libraries(torso_gazebo
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OROCOS-RTT_LIBRARIES} 
  ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} 
  ${OROCOS-RTT_RTT-TRANSPORT-CORBA_LIBRARY}
  ${USE_OROCOS_LIBRARIES}
  ${Boost_LIBRARIES}
)

set_target_properties(barrett_hand_gazebo PROPERTIES COMPILE_FLAGS -DRTT_COMPONENT)
target_link_libraries(barrett_hand_gazebo
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OROCOS-RTT_LIBRARIES} 
  ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} 
  ${OROCOS-RTT_RTT-TRANSPORT-CORBA_LIBRARY}
  ${USE_OROCOS_LIBRARIES}
  ${Boost_LIBRARIES}
)

set_target_properties(barrett_tactile_gazebo PROPERTIES COMPILE_FLAGS -DRTT_COMPONENT)
target_link_libraries(barrett_tactile_gazebo
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OROCOS-RTT_LIBRARIES} 
  ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} 
  ${OROCOS-RTT_RTT-TRANSPORT-CORBA_LIBRARY}
  ${USE_OROCOS_LIBRARIES}
  ${Boost_LIBRARIES}
)

set_target_properties(optoforce_gazebo PROPERTIES COMPILE_FLAGS -DRTT_COMPONENT)
target_link_libraries(optoforce_gazebo
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OROCOS-RTT_LIBRARIES} 
  ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} 
  ${OROCOS-RTT_RTT-TRANSPORT-CORBA_LIBRARY}
  ${USE_OROCOS_LIBRARIES}
  ${Boost_LIBRARIES}
)

set_target_properties(ft_sensor_gazebo PROPERTIES COMPILE_FLAGS -DRTT_COMPONENT)
target_link_libraries(ft_sensor_gazebo
  ${GAZEBO_LIBRARIES}
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
