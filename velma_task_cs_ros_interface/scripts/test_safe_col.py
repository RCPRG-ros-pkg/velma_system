#!/usr/bin/env python

## Runs test for safe_col mode.
# @ingroup integration_tests
# @file test_safe_col.py
# @namespace scripts.test_safe_col Integration test

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy

from velma_common import *
from rcprg_ros_utils import exitError

if __name__ == "__main__":

    rospy.init_node('test_cimp', anonymous=False)

    rospy.sleep(1)

    print "Running python interface for Velma..."
    velma = VelmaInterface()
    print "Waiting for VelmaInterface initialization..."
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"

    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(1)

    print "Motors must be enabled every time after the robot enters safe state."
    print "If the motors are already enabled, enabling them has no effect."
    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(14)

    velma.switchToSafeColBehavior()

    rospy.sleep(0.5)
    exit(0)
