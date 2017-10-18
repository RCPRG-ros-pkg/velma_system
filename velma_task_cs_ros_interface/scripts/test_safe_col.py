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

    velma = VelmaInterface("/velma_task_cs_ros_interface")
    print "waiting for VelmaInterface init..."
    if not velma.waitForInit(timeout_s=20):
        print "could not initialize VelmaInterface"
        exitError(1)
    print "VelmaInterface init ok"


    if velma.enableMotors() != 0:
        exitError(14)

    print "sending head pan START_HOMING command"
    velma.startHomingHP()
    if velma.waitForHP() != 0:
        exitError(14)

    print "sending head tilt START_HOMING command"
    velma.startHomingHT()
    if velma.waitForHT() != 0:
        exitError(15)

    velma.switchToSafeColBehavior()

    rospy.sleep(1)
    exit(0)
