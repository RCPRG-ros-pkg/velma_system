cmake_minimum_required(VERSION 2.8)
project(velma_common)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

catkin_install_python(PROGRAMS
    scripts/int_markers_cimp
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

