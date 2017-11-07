# velma_core_ve_body {#velma_core_ve_body_readme}

This package defines and implements virtual effector of Velma core agent.

It provides ROS launch file:
 * velma_core_ve_body.launch - runs the subsystem; arguments:
   * LOG_LEVEL - Orocos log level, for details type `deployer --help`
   * debug - if `true`, the deployer is executed in gdb.
   * cset - if `true`, the deployer is executed on one CPU core, this works only on RT-linux

