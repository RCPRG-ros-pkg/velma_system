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

    rospy.init_node('cimp_test', anonymous=True)

    rospy.sleep(1)

    velma = VelmaInterface("/velma_task_cs_ros_interface")
    print "waiting for init..."

    velma.waitForInit()
    print "init ok"


    init_q_joint_names = ["torso_0_joint", "right_arm_0_joint", "right_arm_1_joint", "right_arm_2_joint", "right_arm_3_joint", "right_arm_4_joint", "right_arm_5_joint", "right_arm_6_joint",
        "left_arm_0_joint", "left_arm_1_joint", "left_arm_2_joint", "left_arm_3_joint", "left_arm_4_joint", "left_arm_5_joint", "left_arm_6_joint"]
    init_q = [0, 0,-1.57,1.57,1.57,0,-1.57,0,0,1.57,-1.57,-1.57,0,1.57,0]

    velma.moveJoint(init_q, init_q_joint_names, 5.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()

    max_wr = PyKDL.Wrench(PyKDL.Vector(10,10,10), PyKDL.Vector(4,4,4))

    print "moving right tool to wrist frame..."
    T_B_Wr = velma.getTf('B', 'Wr')
    velma.moveCartImpRight([T_B_Wr], [0.5], [PyKDL.Frame()], [0.5], None, None, max_wr, start_time=0.5)
    result = velma.waitForEffectorRight()
    if result != CartesianTrajectoryResult.SUCCESSFUL:
        print result
        raise Exception()

    print "moving left tool to wrist frame..."
    T_B_Wl = velma.getTf('B', 'Wl')
    velma.moveCartImpLeft([T_B_Wl], [0.5], [PyKDL.Frame()], [0.5], None, None, max_wr, start_time=0.5)
    result = velma.waitForEffectorLeft()
    if result != CartesianTrajectoryResult.SUCCESSFUL:
        print result
        raise Exception()

    exit(0)

    print "moving right tool..."
    T_B_Gr = velma.getTf('B', 'Gr')
    T_Wr_Gr = velma.getTf('Wr', 'Gr')
    velma.moveCartImpRight([T_B_Gr], [4.0], [T_Wr_Gr], [4.0], None, None, max_wr, start_time=0.5)

    result = velma.waitForEffectorRight()
    if result != CartesianTrajectoryResult.SUCCESSFUL:
        print result
        raise Exception()

    print "moving left tool..."
    T_B_Gl = velma.getTf('B', 'Gl')
    T_Wl_Gl = velma.getTf('Wl', 'Gl')
    velma.moveCartImpLeft([T_B_Gl], [4.0], [T_Wl_Gl], [4.0], None, None, max_wr, start_time=0.5)

    result = velma.waitForEffectorLeft()
    if result != CartesianTrajectoryResult.SUCCESSFUL:
        print result
        raise Exception()









    print "moving right arm to current pose"
    T_B_Trd = velma.getTf("B", "Wr")
    T_B_Trd_init = copy.copy(T_B_Trd)
    T_B_Tld_init = copy.copy(velma.getTf("B", "Wl"))
    velma.moveEffectorRight(T_B_Trd, 0.5, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, stamp=None, path_tol=None)
    velma.waitForEffectorRight()

    print "moving right arm to another pose"
    T_B_Trd = velma.getTf("B", "Wr")
    T_B_Trd = PyKDL.Frame(PyKDL.Vector(0,0,0.1)) * T_B_Trd
    velma.moveEffectorRight(T_B_Trd, 3.0, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, stamp=None, path_tol=None)
    velma.waitForEffectorRight()

    print "moving right arm to initial pose"
    velma.moveEffectorRight(T_B_Trd_init, 3.0, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, stamp=None, path_tol=None)
    velma.waitForEffectorRight()

    print "moving right arm to another pose (error - too fast)"
    T_B_Trd = velma.getTf("B", "Wr")
    T_B_Trd = PyKDL.Frame(PyKDL.Vector(0,0,1.1)) * T_B_Trd
    velma.moveEffectorRight(T_B_Trd, 0.1, PyKDL.Wrench(PyKDL.Vector(50,50,50), PyKDL.Vector(50,50,50)), start_time=0.5, stamp=None, path_tol=PyKDL.Twist(PyKDL.Vector(0.04, 0.04, 0.04), PyKDL.Vector(0.1, 0.1, 0.1)))
    velma.waitForEffectorRight()

    print "waiting 2 seconds..."
    rospy.sleep(2)

    print "moving right arm to initial pose"
    velma.moveEffectorRight(T_B_Trd_init, 3.0, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, stamp=None, path_tol=None)
    velma.waitForEffectorRight()

    print "moving arms to self-collision pose"
    T_B_Trd = PyKDL.Frame(PyKDL.Vector(0.4, 0.1, 1.2))
    T_B_Tld = PyKDL.Frame(PyKDL.Rotation.RotZ(180.0/180.0*math.pi), PyKDL.Vector(0.4, -0.1, 1.2))
    velma.moveEffectorRight(T_B_Trd, 5.0, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, stamp=None, path_tol=None)
    velma.moveEffectorLeft(T_B_Tld, 5.0, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, stamp=None, path_tol=None)
    velma.waitForEffectorRight()
    velma.waitForEffectorLeft()

    print "waiting 2 seconds..."
    rospy.sleep(2)

    print "moving arms to initial pose"
    velma.moveEffectorRight(T_B_Trd_init, 5.0, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, stamp=None, path_tol=None)
    velma.moveEffectorLeft(T_B_Tld_init, 5.0, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, stamp=None, path_tol=None)
    velma.waitForEffectorRight()
    velma.waitForEffectorLeft()

