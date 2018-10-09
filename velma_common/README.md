# velma_common {#velma_common_readme}

This package provides python interface to WUT Velma Robot, python scripts and ROS launch files:
 * ar_track_kinect_50.launch - runs ROS node for AR visual marker recognition and localization; arguments:
   * *marker_size*
   * *max_new_marker_error*
   * *max_track_error*
   * *cam_image_topic*
   * *cam_info_topic*
   * *output_frame*
 * kinect_hw.launch - runs HW kinect ROS nodes
 * octomap_offline_server.launch - runs offline octomap server with predefined (saved) map file; arguments:
   * *octomap_file* - file path for preloaded octomap file. The map sile can be saved using `rosrun octomap_server octomap_saver`
 * octomap_server.launch - runs the following nodes:
   * point cloud filter that filters out robot links
   * online octomap server that uses filtered point cloud to build occupancy map of the environment
   * octomap filter that adds to the occupancy map unexplored space information

  arguments:
   * *use_hw_kinect* - `true` if real HW kinect is used, `false` for simulated kinect
   * *octomap_file* - file path for preloaded octomap file

 * show_collisions.launch - runs script that vizualises collision objects and collisions between them
 * show_kinect_frustum.launch - runs script that visualizes frustum of kinect mounted on head of Velma
 * velma_system.launch - runs all core agent and task agent subsystems
   * *GAZEBO_MODEL_PATH* - please refer to Gazebo documentation
   * *GAZEBO_RESOURCE_PATH* - please refer to Gazebo documentation
   * *use_kinect* - `true` or `false`: enable or disable kinect simulation
   * *world_name* - world file name with respect to GAZEBO_RESOURCE_PATH
   * *profile* - physics profile as defined in world file
 * int_markers_cimp - provides interactive 6D pose marker that ca be used to move end-effectors.
 * show_joints.launch - show joints and their limits as rviz visual markers
