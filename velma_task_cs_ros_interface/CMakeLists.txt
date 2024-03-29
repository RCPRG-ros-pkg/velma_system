cmake_minimum_required(VERSION 2.8.3)
project(velma_task_cs_ros_interface)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  rtt_rosparam
  rtt_subsystem
  velma_core_cs_task_cs_msgs
  eigen_conversions
  rtt_actionlib
  common_interfaces
  subsystem_common
  controller_common
  rcprg_planner
  behavior_switch_action_msgs
  )

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
#  DEPENDS rtt_ros std_msgs message_runtime rtt_roscomm velma_core_cs_task_cs_interface_msgs
#)

include_directories(${USE_OROCOS_INCLUDE_DIRS} include)
### Orocos Targets ###

ros_generate_rtt_master()

orocos_component(velma_task_cs_ros_interface
    src/velma_task_cs_ros_interface.cpp
    src/components/cart_imp_action.cpp
    src/components/InternalSpaceSplineTrajectoryAction.cpp
    src/components/HeadInternalSpaceSplineTrajectoryAction.cpp
    src/components/vector_split15_2_8_8.cpp
    src/components/relax_action.cpp
)
target_link_libraries(velma_task_cs_ros_interface ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} rtt-${PROJECT_NAME}-master)

#orocos_service(VelmaCoreVeBodyMaster
#    src/master_service.cpp
#)
#target_link_libraries(VelmaCoreVeBodyMaster ${catkin_LIBRARIES})

### Orocos Package Exports and Install Targets ###

# Export package information (replaces catkin_package() macro) 
orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros std_msgs message_runtime rtt_roscomm rtt_actionlib
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  )

catkin_install_python(PROGRAMS
    scripts/initialize_robot.py
    scripts/reset_head_position.py
    scripts/set_gripper_configuration.py
    scripts/switch_to_jimp.py
    scripts/test_all.py
    scripts/test_cimp_imp.py
    scripts/test_cimp_pose.py
    scripts/test_cimp_tool.py
    scripts/test_grippers.py
    scripts/test_head.py
    scripts/test_head_complex.py
    scripts/test_jimp.py
    scripts/test_jimp_endless.py
    scripts/test_relax.py
    scripts/test_jimp_planning.py
    scripts/test_jimp_planning_attached.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#orocos_install_headers(
#  DIRECTORY include/${PROJECT_NAME}/
#)
