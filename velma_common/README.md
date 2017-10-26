# velma_common {#velma_common_readme}

This package provides python interface to WUT Velma Robot, python scripts and ROS launch files:
 * ar_track_kinect_50.launch - runs ROS node for AR visual marker recognition and localization
 * kinect_hw.launch - runs HW kinect ROS nodes
 * octomap_offline_server.launch - runs offline octomap server with predefined (saved) map file
 * octomap_server.launch - runs:
   - point cloud filter that filters out robot links
   - online octomap server that uses filtered point cloud to build occupancy map of the environment
   - octomap filter that adds to the occupancy map unexplored space
 * show_collisions.launch - runs script that vizualises collision objects and collisions between them
 * show_kinect_frustrum.launch - runs script that visualizes frustrum of kinect mounted on head of Velma

