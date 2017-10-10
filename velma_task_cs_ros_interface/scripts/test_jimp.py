#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')

import rospy
import math
import PyKDL

from velma_common.velma_interface import *
from planner.planner import *

if __name__ == "__main__":

    rospy.init_node('jimp_test', anonymous=True)

    rospy.sleep(1)

    velma = VelmaInterface("/velma_task_cs_ros_interface")
    print "waiting for init..."

    velma.waitForInit()
    print "init ok"

#
#
#
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

    print "moving to current position"
    js = velma.getLastJointState()
    joint_names = []
    q_dest = []
    for joint_name in js[1]:
        joint_names.append(joint_name)
        q_dest.append(js[1][joint_name])

    velma.moveJoint(q_dest, joint_names, 1.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()

    print "moving to position 0 (too fast - error)"
    q_dest_0 = []
    for joint_name in js[1]:
        q_dest_0.append(0)

    velma.moveJoint(q_dest_0, joint_names, 0.1, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()

    rospy.sleep(2.0)

    if velma.enableMotors() != 0:
        exitError(14)

    print "waiting 2 seconds..."
    rospy.sleep(2)

    print "moving to position 0 (slowly)"
    velma.moveJoint(q_dest_0, joint_names, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()

    print "moving to initial position"
    velma.moveJoint(q_dest, joint_names, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()

