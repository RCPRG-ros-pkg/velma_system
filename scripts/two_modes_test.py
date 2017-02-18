#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')

import sys
import rospy
import math
import copy
import tf

from std_msgs.msg import ColorRGBA
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from tf.transformations import * 
import tf_conversions.posemath as pm
import PyKDL
from cartesian_trajectory_msgs.msg import *
import actionlib

from velma_common.velma_interface import *

if __name__ == "__main__":

    rospy.init_node('jimp_test', anonymous=True)

    prefix = "right"

    rospy.sleep(1)


    velma = VelmaInterface("/velma_task_cs_ros_interface")
    print "waiting for init..."

    velma.waitForInit()
    print "init ok"

    q_map = {'torso_0_joint':0.0,
        'right_arm_0_joint':-0.3,
        'right_arm_1_joint':-1.57,
        'right_arm_2_joint':1.57,
        'right_arm_3_joint':1.57,
        'right_arm_4_joint':0.0,
        'right_arm_5_joint':-1.57,
        'right_arm_6_joint':0.0,
        'left_arm_0_joint':0.3,
        'left_arm_1_joint':1.57,
        'left_arm_2_joint':-1.57,
        'left_arm_3_joint':-1.7,
        'left_arm_4_joint':0.0,
        'left_arm_5_joint':1.57,
        'left_arm_6_joint':0.0}

    joint_names = []
    for joint_name in q_map:
        joint_names.append(joint_name)

#    js = velma.getLastJointState()
#    joint_names = []
#    q_dest = []
#    for joint_name in js[1]:
#        joint_names.append(joint_name)
    q_dest = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    velma.moveJoint(q_dest, joint_names, 10.0, start_time=0.5)
    velma.waitForJoint()

#    joint_names = []
#    q_dest = []
#    for joint_name in js[1]:
#        joint_names.append(joint_name)
#        q_dest.append(js[1][joint_name])

    q_dest = []
    for joint_name in joint_names:
        q_dest.append(q_map[joint_name])
    velma.moveJoint(q_dest, joint_names, 10.0, start_time=0.5)
    velma.waitForJoint()

    T_B_Trd = velma.getTf("B", "Wr")
    T_B_Trd = T_B_Trd * PyKDL.Frame(PyKDL.Vector(0.1,0,0))
    velma.moveEffectorRight(T_B_Trd, 5.0, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, stamp=None, path_tol=None)
    velma.waitForEffectorRight()

    velma.moveJoint(q_dest, joint_names, 10.0, start_time=0.5)
    velma.waitForJoint()

    T_B_Tld = velma.getTf("B", "Wl")
    T_B_Tld = T_B_Tld * PyKDL.Frame(PyKDL.Vector(0.1,0,0))
    velma.moveEffectorLeft(T_B_Trd, 5.0, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, stamp=None, path_tol=None)
    velma.waitForEffectorLeft()

    velma.moveJoint(q_dest, joint_names, 10.0, start_time=0.5)
    velma.waitForJoint()


