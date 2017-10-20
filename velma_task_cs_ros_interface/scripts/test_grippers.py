#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import math
from velma_common.velma_interface import *

def exitError(code):
    if code == 0:
        print "OK"
        exit(0)
    print "ERROR:", code
    exit(code)

if __name__ == "__main__":

    rospy.init_node('grippers_test', anonymous=True)

    rospy.sleep(1)

    velma = VelmaInterface()
    print "waiting for init..."

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

    if velma.enableMotors() != 0:
        exitError(14)

    print "reset left"
    velma.resetHandLeft()
    if velma.waitForHandLeft() != 0:
        exitError(2)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandLeftCurrentConfiguration(), [0,0,0,0]):
        exitError(3)

    print "reset right"
    velma.resetHandRight()
    if velma.waitForHandRight() != 0:
        exitError(4)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), [0,0,0,0]):
        exitError(5)

    dest_q = [90.0/180.0*math.pi,0,0,0]
    print "move left:", dest_q
    velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandLeft() != 0:
        exitError(6)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandLeftCurrentConfiguration(), dest_q):
        print velma.getHandLeftCurrentConfiguration(), dest_q
        exitError(7)

    dest_q = [90.0/180.0*math.pi,0,0,0]
    print "move right:", dest_q
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(8)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
        exitError(9)

    dest_q = [0,90.0/180.0*math.pi,0,180.0/180.0*math.pi]
    print "move right:", dest_q
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(10)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
        exitError(11)

    print "reset left"
    velma.resetHandLeft()
    if velma.waitForHandLeft() != 0:
        exitError(12)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandLeftCurrentConfiguration(), [0,0,0,0]):
        exitError(13)

    print "reset right"
    velma.resetHandRight()
    if velma.waitForHandRight() != 0:
        exitError(14)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), [0,0,0,0]):
        exitError(15)

    exitError(0)

