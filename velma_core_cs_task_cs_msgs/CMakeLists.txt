cmake_minimum_required(VERSION 2.8.6)

project(velma_core_cs_task_cs_msgs)

find_package(catkin REQUIRED
  COMPONENTS std_msgs geometry_msgs common_interfaces message_generation actionlib_msgs cartesian_trajectory_msgs control_msgs)

add_message_files(FILES
  Command.msg
  CommandCartImp.msg
  CommandCartImpTrjImp.msg
  CommandCartImpTrjPose.msg
  CommandJntImp.msg
  CommandHead.msg
  VelmaJointTrajectoryPoint.msg
  VelmaHeadTrajectoryPoint.msg
  Status.msg
  StatusCartImp.msg
  StatusJntImp.msg
)
  

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  cartesian_trajectory_msgs
  control_msgs
  )


catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs message_runtime cartesian_trajectory_msgs control_msgs)
