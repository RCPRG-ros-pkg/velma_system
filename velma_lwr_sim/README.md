# velma_lwr_sim

This package defines and implements real-time simulated real effector (LWR) of Velma core agent.
It does *NOT* use Gazebo.

It provides ROS launch files:
 * velma_sim_re_lwr_l.launch - runs the simulated RT subsystem for left arm; arguments:
   * LOG_LEVEL - Orocos log level, for details type `deployer --help`
   * debug - if `true`, the deployer is executed in gdb.
   * cset - if `true`, the deployer is executed on one CPU core, this works only on RT-linux
 * velma_sim_re_lwr_r.launch - runs the simulated RT subsystem for right arm; arguments are the same as above

