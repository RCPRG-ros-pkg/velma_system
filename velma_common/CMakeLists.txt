cmake_minimum_required(VERSION 2.8)
project(velma_common)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  )

catkin_install_python(PROGRAMS
    scripts/int_markers_cimp
    scripts/int_markers_jimp
    scripts/reset_shm_comm
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

