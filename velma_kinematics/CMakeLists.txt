cmake_minimum_required(VERSION 2.8)
project(velma_kinematics)

find_package(catkin REQUIRED COMPONENTS
#  pcl_conversions
#  pcl_ros
  eigen_conversions
  sensor_msgs
  rcprg_ros_utils
  collision_convex_model
  kin_dyn_model
  moveit_core
  moveit_ros_planning_interface
#  moveit_ros_perception
  rcprg_planner
)

find_package(OCTOMAP REQUIRED)
find_package(orocos_kdl REQUIRED)
#find_package(OpenCV REQUIRED)
find_package(kdl_parser REQUIRED)

catkin_package(INCLUDE_DIRS include
	DEPENDS OCTOMAP EIGEN3 orocos_kdl kdl_parser
  LIBRARIES velma_kinematics)

catkin_python_setup()

link_directories(${orocos_kdl_LIBRARY_DIRS} ${OCTOMAP_LIBRARY_DIRS})

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} include)

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file

include_directories(${USE_OROCOS_INCLUDE_DIRS} include)
ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)
### Orocos Targets ###

add_library(velma_kinematics
  src/velma_kinematics.cpp
  src/velma_state_validator.cpp)
target_link_libraries(velma_kinematics ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES} ${OCTOMAP_LIBRARIES})

### Orocos Package Exports and Install Targets ###
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_executable(test_ik
  src/test_ik.cpp
  src/velma_kinematics.cpp
  src/velma_state_validator.cpp)
target_link_libraries(test_ik
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
#  ${OpenCV_LIBS}
  ${kdl_parser_LIBRARIES})

catkin_install_python(PROGRAMS
    scripts/test_velma_ik_geom.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
