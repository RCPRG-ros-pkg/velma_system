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

    print "moving to initial position"
    q_dest = []
    for joint_name in joint_names:
        q_dest.append(q_map[joint_name])
    velma.moveJoint(q_dest, joint_names, 3.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()

    rospy.sleep(1)

    print "moving to self-collision position"
    q_map_col = copy.copy(q_map)
    q_map_col['right_arm_5_joint'] = 1.0
    q_dest = []
    for joint_name in joint_names:
        q_dest.append(q_map_col[joint_name])
    velma.moveJoint(q_dest, joint_names, 3.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()

    exit(0)

    print "moving to current position"
    js = velma.getLastJointState()
    q_map_current = js[1]
    q_dest = []
    for name in joint_names:
        q_dest.append( q_map_current[name] )
    velma.moveJoint(q_dest, joint_names, 2.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()


    print "moving to position 0"
    q_dest = [0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    velma.moveJoint(q_dest, joint_names, 6.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()

    if True:
        print "moving to osc. position"
        q_dest_osc = [-0.15214130720857572, 0.10515184331308991, -1.2467876847358885, 0.9126682266649763, 0.878948249832275, 0.006743614987513767, -1.597927382059243, 1.1084458395050156,
            0.17617247391228125, 0.8291781171997458, -0.9786058106846747, -1.0229772583291257, -0.4661064923246427, 1.6189015800736866, -0.8543317202407017]
        velma.moveJoint(q_dest_osc, joint_names, 6.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
        velma.waitForJoint()

        exit(0)

#    exit(0)

    rospy.sleep(2)

    print "moving to initial position"
    q_dest = []
    for joint_name in joint_names:
        q_dest.append(q_map[joint_name])
    velma.moveJoint(q_dest, joint_names, 6.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()

    rospy.sleep(2)

    print "moving right arm in cimp mode"
    T_B_Trd = velma.getTf("B", "Wr")
    T_B_Trd = T_B_Trd * PyKDL.Frame(PyKDL.Vector(0.1,0,0))
    velma.moveEffectorRight(T_B_Trd, 3.0, PyKDL.Wrench(PyKDL.Vector(10,10,10), PyKDL.Vector(10,10,10)), start_time=0.5, stamp=None, path_tol=None)
    velma.waitForEffectorRight()

    print "moving to initial position"
    velma.moveJoint(q_dest, joint_names, 3.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()

    print "moving left arm in cimp mode"
    T_B_Tld = velma.getTf("B", "Wl")
    T_B_Tld = T_B_Tld * PyKDL.Frame(PyKDL.Vector(0.1,0,0))
    velma.moveEffectorLeft(T_B_Tld, 3.0, PyKDL.Wrench(PyKDL.Vector(10,10,10), PyKDL.Vector(10,10,10)), start_time=0.5, stamp=None, path_tol=None)
    velma.waitForEffectorLeft()

    print "moving to initial position"
    velma.moveJoint(q_dest, joint_names, 3.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()


