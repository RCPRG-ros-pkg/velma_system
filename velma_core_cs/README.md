# velma_core_cs {#velma_core_cs_readme}

This package defines and implements control subsystem of Velma core agent.

It provides ROS launch file:
 * velma_core_cs.launch - runs the subsystem; arguments:
   * LOG_LEVEL - Orocos log level, for details type `deployer --help`
   * debug - if `true`, the deployer is executed in gdb.
   * cset - if `true`, the deployer is executed on one CPU core, this works only on RT-linux
   * load_robot_description - if `true`, the launch file loads robot description from urdf and runs robot_state_publisher

