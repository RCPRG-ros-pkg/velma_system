#!/usr/bin/env python

## Runs test for simple head motions.
# @ingroup integration_tests
# @file test_head.py
# @namespace scripts.test_head Integration test

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')

import rospy
import copy

from velma_common import *
from rcprg_ros_utils import exitError

if __name__ == "__main__":

    rospy.init_node('head_test', anonymous=True)

    rospy.sleep(0.5)

    print "This test/tutorial executes simple motions"\
        " of head."

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
        exitError(2)

    print "Moving to the current position..."
    js_start = velma.getLastJointState()
    velma.moveJoint(js_start[1], 0.5, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(3)

    print "moving head to position: 0"
    q_dest = (0,0)
    velma.moveHead(q_dest, 2.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(4)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(5)

    print "moving head to position: left"
    q_dest = (1.56, 0)
    velma.moveHead(q_dest, 2.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(6)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(7)

    print "moving head to position: right"
    q_dest = (-1.56, 0)
    velma.moveHead(q_dest, 4.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(8)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(9)

    print "moving head to position: up"
    q_dest = (0, -0.99)
    velma.moveHead(q_dest, 4.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(10)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(11)

    print "moving head to position: down"
    q_dest = (0, 1.29)
    velma.moveHead(q_dest, 4.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(12)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(13)

    print "moving head to position: 0"
    q_dest = (0,0)
    velma.moveHead(q_dest, 2.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(14)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(15)

    exitError(0)

