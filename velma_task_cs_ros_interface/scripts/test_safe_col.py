#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy

from velma_common.velma_interface import *

def exitError(code):
    if code == 0:
        print "OK"
        exit(0)
    print "ERROR:", code
    exit(code)

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

    print "Motors must be enabled every time after the robot enters safe state."
    print "If the motors are already enabled, enabling them has no effect."
    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(14)

    velma.switchToSafeColBehavior()

    rospy.sleep(0.5)
    exit(0)
