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
    scripts/int_markers_cimp.py
    scripts/int_markers_jimp.py
    scripts/reset_shm_comm.py
    scripts/velma_look_at_action_server.py
    scripts/odom_localization.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

