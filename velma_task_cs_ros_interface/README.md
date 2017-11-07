# velma_task_cs_ros_interface {#velma_task_cs_ros_interface_readme}

This package defines and implements control subsystem of Velma task agent.

It provides ROS launch file:
 * velma_task_cs_ros_interface.launch - runs the subsystem
   * LOG_LEVEL - Orocos log level, for details type `deployer --help`
   * debug - if `true`, the deployer is executed in gdb.
   * cset - if `true`, the deployer is executed on one CPU core, this works only on RT-linux

It also provides various scripts for integration tests and tutorials.

