project(velma_core_cs)

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  rtt_rosclock
  rtt_subsystem
  velma_core_cs_task_cs_msgs
  velma_core_cs_ve_body_msgs
  eigen_conversions
  common_interfaces
  subsystem_common
  controller_common
  planer_utils
  common_core_cs_components
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
#  DEPENDS rtt_ros std_msgs message_runtime rtt_roscomm velma_core_cs_task_cs_interface_msgs
#)

include_directories(${USE_OROCOS_INCLUDE_DIRS} include)
### Orocos Targets ###

ros_generate_rtt_master()

orocos_component(velma_core_cs
    src/velma_core_cs.cpp
    src/predicates.cpp
    src/components/idle.cpp
    src/components/safe.cpp
    src/components/cart_imp.cpp
    src/components/jnt_imp.cpp
    src/components/fcl.cpp
    src/components/cartesian_interpolator.cpp
    src/components/cartesian_impedance_interpolator.cpp
    src/components/InternalSpaceSplineTrajectoryGeneratorVelmaBody.cpp
    src/components/InternalSpaceSplineTrajectoryGeneratorVelmaHead.cpp
    src/components/vector_dummy.cpp
    src/components/double_joint_constraint.cpp
    src/components/collision_detector.cpp
    src/components/double_joint_collision_detector.cpp
    src/components/vector_concate1_1_0_0.cpp
    src/components/vector_split1_7_7_0.cpp
    src/components/vector_concate1_7_7_0.cpp
    src/components/vector_concate15_2_8_8.cpp
    src/components/input_converter.cpp
    src/components/output_converter.cpp
)
target_link_libraries(velma_core_cs ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} rtt-${PROJECT_NAME}-master)

### Orocos Package Exports and Install Targets ###

# Export package information (replaces catkin_package() macro) 
orocos_generate_package(
  INCLUDE_DIRS include
#  LIBRARIES velma_core_cs
  DEPENDS rtt_ros std_msgs message_runtime rtt_roscomm velma_core_cs_task_cs_interface_msgs common_core_cs_components
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  )


#orocos_install_headers(
#  DIRECTORY include/${PROJECT_NAME}/
#)
