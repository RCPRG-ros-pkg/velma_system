#!/usr/bin/env python

# Copyright (c) 2015, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Dawid Seredynski
#

import roslib
roslib.load_manifest('velma_sim_gazebo')

import rospy
import tf

from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from barrett_hand_controller_msgs.msg import *
from cartesian_trajectory_msgs.msg import *
from visualization_msgs.msg import *
import actionlib
from actionlib_msgs.msg import *
from threading import Lock

from tf.transformations import * 
import tf_conversions.posemath as pm
from tf2_msgs.msg import *
import force_control_msgs.msg
from control_msgs.msg import *

import PyKDL
import math
import numpy as np
import copy
import matplotlib.pyplot as plt
import thread
from velma_common.velma_interface import VelmaInterface
import random
import velma_common.velmautils as velmautils
import itertools
import operator
import rospkg
from scipy import optimize

def makeWrench(f_x, f_y, f_z, t_x, t_y, t_z):
    return PyKDL.Wrench(PyKDL.Vector(f_x,f_y,f_z), PyKDL.Vector(t_x,t_y,t_z))

def calcOffsetStiffWr(stiff, wr):
    return PyKDL.Twist(
        PyKDL.Vector(wr.force.x() / stiff.force.x(), wr.force.y() / stiff.force.y(), wr.force.z() / stiff.force.z()),
        PyKDL.Vector(wr.torque.x() / stiff.torque.x(), wr.torque.y() / stiff.torque.y(), wr.torque.z() / stiff.torque.z()))

class VelmaTest:
    """
Class for the Control Subsystem behaviour: cabinet door opening.
"""

    def __init__(self, pub_marker=None):
        self.pub_marker = velmautils.MarkerPublisher()

    def testJntImp(self, velma):
        print "switching to joint impedance..."
        if not velma.switchToJntImp():
            raise Exception()
        print "done."

        print "current behaviour: ", velma.getControllerBehaviourName()

        # the joint trajectory
        joint_names = ['torso_0_joint', 'left_arm_0_joint', 'left_arm_1_joint', 'left_arm_2_joint',
            'left_arm_3_joint', 'left_arm_4_joint', 'left_arm_5_joint', 'left_arm_6_joint',
            'right_arm_0_joint', 'right_arm_1_joint', 'right_arm_2_joint', 'right_arm_3_joint',
            'right_arm_4_joint', 'right_arm_5_joint', 'right_arm_6_joint']

        dest_q = [ (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
            (0.0010684921160319114, -0.024792216580639174, 1.8305036505955312, -1.8259925552720098,
            -1.5517390989446727, 0.2730274443003502, 1.5860233121482932, -0.2502297176286951,
            0.02262283968089819, -1.8302122743535556, 1.827165182233292, 1.553072907701446,
            -0.27115008899917403, -1.5863265032757212, 0.2493928526470015) ]

        print "executing joint trajectory..."
        if not velma.moveJointTraj([dest_q, None, None, (10.0, 10.0)], joint_names, start_time=0.2):
            raise Exception()
        result = velma.waitForJoint()
        if result != FollowJointTrajectoryResult.SUCCESSFUL:
            print result
            raise Exception()
        print "done."

    def testTool(self, velma):
        T_B_Wr = velma.getTf('B', 'Wr')
        T_B_Wl = velma.getTf('B', 'Wl')

        T_Wr_Gr = velma.getTf('Wr', 'Gr')
        T_Wl_Gl = velma.getTf('Wl', 'Gl')

        if not velma.switchToJntImp():
            raise Exception()
        velma.moveToolRight(velma.getTf('Wr', 'Gr'), 0.1)
        velma.moveToolLeft(velma.getTf('Wl', 'Gl'), 0.1)
        if velma.waitForToolLeft() != 0 or velma.waitForToolRight() != 0:
            raise Exception()

        if not velma.switchToCartImp():
            raise Exception()

        rospy.sleep(1)

        diff_r = PyKDL.diff(T_B_Wr * T_Wr_Gr, velma.getTf('B', 'Tr'))
        diff_l = PyKDL.diff(T_B_Wl * T_Wl_Gl, velma.getTf('B', 'Tl'))

        print "right tool diff: ", diff_r.vel.Norm(), diff_r.rot.Norm()
        print "left tool diff: ", diff_l.vel.Norm(), diff_l.rot.Norm()
        if diff_r.vel.Norm() > 0.001 or diff_r.rot.Norm() > 0.001 or diff_l.vel.Norm() > 0.001 or diff_l.rot.Norm() > 0.001:
            raise Exception()

    def spin(self):
        print "creating interface for Velma..."
        # create the interface for Velma robot
        velma = VelmaInterface()
        velma.waitForInit()
        if rospy.is_shutdown():
            raise Exception()
        print "done."

        print "current behaviour: ", velma.getControllerBehaviourName()

        self.testJntImp(velma)
        self.testTool(velma)

        if not velma.switchToJntImp():
            raise Exception()
        velma.moveToolRight(velma.getTf('Wr', 'Gr'), 0.1)
        velma.moveToolLeft(velma.getTf('Wl', 'Gl'), 0.1)
        if velma.waitForToolLeft() != 0 or velma.waitForToolRight() != 0:
            raise Exception()

        if not velma.switchToCartImp():
            raise Exception()

        T_B_Grd = PyKDL.Frame(PyKDL.Rotation.RotY(90.0/180.0*math.pi), PyKDL.Vector(0.7, -0.6, 1.2))
        if not velma.moveEffectorRight(T_B_Grd, 5.0, PyKDL.Wrench(PyKDL.Vector(10,10,10), PyKDL.Vector(4,4,4)), start_time=0.1, stamp=None, path_tol=None):
            raise Exception()
        result = velma.waitForEffectorRight()
        if result != CartesianTrajectoryResult.SUCCESSFUL:
            print result
            raise Exception()

        T_B_Gld = PyKDL.Frame(PyKDL.Rotation.RotY(90.0/180.0*math.pi) * PyKDL.Rotation.RotZ(180.0/180.0*math.pi), PyKDL.Vector(0.7, 0.6, 1.2))
        if not velma.moveEffectorLeft(T_B_Gld, 5.0, PyKDL.Wrench(PyKDL.Vector(10,10,10), PyKDL.Vector(4,4,4)), start_time=0.1, stamp=None, path_tol=None):
            raise Exception()
        result = velma.waitForEffectorLeft()
        if result != CartesianTrajectoryResult.SUCCESSFUL:
            print result
            raise Exception()

        T_B_Grd = PyKDL.Frame(PyKDL.Rotation.RotZ(90.0/180.0*math.pi) * PyKDL.Rotation.RotY(90.0/180.0*math.pi), PyKDL.Vector(0.6, -0.3, 1.2))
        if not velma.moveEffectorRight(T_B_Grd, 5.0, PyKDL.Wrench(PyKDL.Vector(10,10,10), PyKDL.Vector(4,4,4)), start_time=0.1, stamp=None, path_tol=None):
            raise Exception()
        T_B_Gld = PyKDL.Frame(PyKDL.Rotation.RotZ(-90.0/180.0*math.pi) * PyKDL.Rotation.RotY(90.0/180.0*math.pi) * PyKDL.Rotation.RotZ(180.0/180.0*math.pi), PyKDL.Vector(0.6, 0.3, 1.2))
        if not velma.moveEffectorLeft(T_B_Gld, 5.0, PyKDL.Wrench(PyKDL.Vector(10,10,10), PyKDL.Vector(4,4,4)), start_time=0.1, stamp=None, path_tol=None):
            raise Exception()
        result = velma.waitForEffectorRight()
        if result != CartesianTrajectoryResult.SUCCESSFUL:
            print result
            raise Exception()
        result = velma.waitForEffectorLeft()
        if result != CartesianTrajectoryResult.SUCCESSFUL:
            print result
            raise Exception()

        T_B_Grd = PyKDL.Frame(PyKDL.Rotation.RotZ(90.0/180.0*math.pi) * PyKDL.Rotation.RotY(90.0/180.0*math.pi), PyKDL.Vector(0.6, -0.05, 1.2))
        if not velma.moveEffectorRight(T_B_Grd, 5.0, PyKDL.Wrench(PyKDL.Vector(10,10,10), PyKDL.Vector(4,4,4)), start_time=0.1, stamp=None, path_tol=None):
            raise Exception()
        T_B_Gld = PyKDL.Frame(PyKDL.Rotation.RotZ(-90.0/180.0*math.pi) * PyKDL.Rotation.RotY(90.0/180.0*math.pi) * PyKDL.Rotation.RotZ(180.0/180.0*math.pi), PyKDL.Vector(0.6, 0.05, 1.2))
        if not velma.moveEffectorLeft(T_B_Gld, 5.0, PyKDL.Wrench(PyKDL.Vector(10,10,10), PyKDL.Vector(4,4,4)), start_time=0.1, stamp=None, path_tol=None):
            raise Exception()
        result = velma.waitForEffectorRight()
        if result != CartesianTrajectoryResult.SUCCESSFUL:
            print result
            raise Exception()
        result = velma.waitForEffectorLeft()
        if result != CartesianTrajectoryResult.SUCCESSFUL:
            print result
            raise Exception()

        # create fcl topics
        pub_fcl_r = rospy.Publisher('/right_arm/fcl_param', force_control_msgs.msg.ForceControl, queue_size=0)
        pub_fcl_l = rospy.Publisher('/left_arm/fcl_param', force_control_msgs.msg.ForceControl, queue_size=0)

        # wait a while to stabilize the robot and the F/T sensor output
        rospy.sleep(2)

        goal = force_control_msgs.msg.ForceControl()
        goal.inertia = force_control_msgs.msg.Inertia(Vector3(20.0, 20.0, 20.0), Vector3(0.5, 0.5, 0.5))
        goal.reciprocaldamping = force_control_msgs.msg.ReciprocalDamping(Vector3(0.1, 0.1, 0.025), Vector3(0.01, 0.01, 0.01))
        goal.wrench = geometry_msgs.msg.Wrench(Vector3(0.0, 0.0, 0.3), Vector3(0.0, 0.0, 0.0))
        goal.twist = geometry_msgs.msg.Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        pub_fcl_r.publish(goal)
        goal.wrench = geometry_msgs.msg.Wrench(Vector3(0.0, 0.0, 0.3), Vector3(0.0, 0.0, 0.0))
        pub_fcl_l.publish(goal)

        if not velma.switchToCartFcl():
            raise Exception()

        prev_T_B_Gr = None
        while not rospy.is_shutdown():
            T_B_Gr = velma.getTf('B', 'Gr')
            if prev_T_B_Gr != None:
                diff = PyKDL.diff(prev_T_B_Gr, T_B_Gr)
                wr = velma.getTransformedFTr()
                vel = diff.vel.Norm()
                rot = diff.rot.Norm()
                print vel, rot
                if vel < 0.003*0.1 and rot < (1.0/180.0*math.pi)*0.1 and wr.force.z() > 0.4:
                    break
            prev_T_B_Gr = T_B_Gr
            rospy.sleep(0.1)

        if not velma.switchToCartImp():
            raise Exception()

        # TODO: tactile sensors, optoforce sensors, head, behavious switching

        return

if __name__ == '__main__':

    rospy.init_node('velma_test')

    task = VelmaTest()

    task.spin()


