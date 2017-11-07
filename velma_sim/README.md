# velma_sim

This package defines and implements real-time simulated real effector (torso and neck) of Velma core agent.
It does *NOT* use Gazebo.

It provides ROS launch files:
 * velma_sim_re.launch - runs the subsystem; arguments:
   * LOG_LEVEL - Orocos log level, for details type `deployer --help`
   * debug - if `true`, the deployer is executed in gdb.
   * cset - if `true`, the deployer is executed on one CPU core, this works only on RT-linux

