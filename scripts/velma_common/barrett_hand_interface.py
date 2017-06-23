#!/usr/bin/env python

# Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
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

import rospy
import tf

import geometry_msgs.msg
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from barrett_hand_msgs.msg import *
from barrett_hand_action_msgs.msg import *
from cartesian_trajectory_msgs.msg import *
from visualization_msgs.msg import *
import actionlib
from actionlib_msgs.msg import *
from trajectory_msgs.msg import *
from control_msgs.msg import *
from threading import Lock
from functools import partial

import tf
from tf import *
from tf.transformations import * 
import tf_conversions.posemath as pm
from tf2_msgs.msg import *
import controller_manager_msgs.srv


import PyKDL
import math
from numpy import *
import numpy as np

import copy

# reference frames:
# G - grasp
# Gr - right grasp
# Gl - left grasp
# Frij - right hand finger i knuckle j

class BarrettHandInterface:
    """
Class used as BarrettHand robot Interface.
"""

    def getLastJointState(self):
        result = None
        self.joint_states_lock.acquire()
        if self.js_pos_history_idx >= 0:
            result = copy.copy(self.js_pos_history[self.js_pos_history_idx])
        self.joint_states_lock.release()
        return result

    def getJointStateAtTime(self, time):
        self.joint_states_lock.acquire()
        if self.js_pos_history_idx < 0:
            self.joint_states_lock.release()
            return None
            
        hist_len = len(self.js_pos_history)
        for step in range(hist_len-1):
            h1_idx = (self.js_pos_history_idx - step - 1) % hist_len
            h2_idx = (self.js_pos_history_idx - step) % hist_len
            if self.js_pos_history[h1_idx] == None or self.js_pos_history[h2_idx] == None:
                self.joint_states_lock.release()
                return None

            time1 = self.js_pos_history[h1_idx][0]
            time2 = self.js_pos_history[h2_idx][0]
            if time > time1 and time <= time2:
                factor = (time - time1).to_sec() / (time2 - time1).to_sec()
                js_pos = {}
                for joint_name in self.js_pos_history[h1_idx][1]:
                    js_pos[joint_name] = self.js_pos_history[h1_idx][1][joint_name] * (1.0 - factor) + self.js_pos_history[h2_idx][1][joint_name] * factor
                self.joint_states_lock.release()
                return js_pos
        self.joint_states_lock.release()
        return None

    def jointStatesCallback(self, data):
        self.joint_states_lock.acquire()
        joint_idx = 0
        for joint_name in data.name:
            self.js_pos[joint_name] = data.position[joint_idx]
            joint_idx += 1

        self.js_pos_history_idx = (self.js_pos_history_idx + 1) % len(self.js_pos_history)
        self.js_pos_history[self.js_pos_history_idx] = (data.header.stamp, copy.copy(self.js_pos))
        self.joint_states_lock.release()

    def waitForInit(self):
        while not rospy.is_shutdown():
            can_break = False
            self.joint_states_lock.acquire()
            if self.js_pos_history_idx >= 0:
                can_break = True
            self.joint_states_lock.release()
            if can_break:
                break
            rospy.sleep(0.1)

    def __init__(self, namespace="/barrett_hand_cs_ros_interface"):

        self.listener = tf.TransformListener();

        # read the joint information from the ROS parameter server
        self.all_joint_names = rospy.get_param(namespace+"/JntPub/joint_names")

        self.js_pos = {}
        self.js_pos_history = []
        for i in range(200):
            self.js_pos_history.append( None )
        self.js_pos_history_idx = -1

        self.last_contact_time = rospy.Time.now()

        self.joint_states_lock = Lock()

        self.action_move_hand_client = {
            'right':actionlib.SimpleActionClient("/right_hand/move_hand", BHMoveAction) }
        self.action_move_hand_client["right"].wait_for_server()

#        self.pub_reset_left = rospy.Publisher("/left_hand/reset_fingers", std_msgs.msg.Empty, queue_size=100)
#        self.pub_reset_right = rospy.Publisher("/right_hand/reset_fingers", std_msgs.msg.Empty, queue_size=100)

        rospy.sleep(1.0)
        
        self.listener_joint_states = rospy.Subscriber('/joint_states', JointState, self.jointStatesCallback)

    def moveHand(self, q, v, t, maxPressure, hold=False, prefix="right"):
        action_goal = BHMoveGoal()
        action_goal.name = [prefix+"_HandFingerOneKnuckleTwoJoint", prefix+"_HandFingerTwoKnuckleTwoJoint", prefix+"_HandFingerThreeKnuckleTwoJoint", prefix+"_HandFingerOneKnuckleOneJoint"]
        action_goal.q = q
        action_goal.v = v
        action_goal.t = t
        action_goal.maxPressure = maxPressure
        if hold == True:
            action_goal.hold = 1
        else:
            action_goal.hold = 0
        action_goal.reset = False
        self.action_move_hand_client[prefix].send_goal(action_goal)

    def resetHand(self, prefix="right"):
        action_goal = BHMoveGoal()
        action_goal.reset = True
        self.action_move_hand_client[prefix].send_goal(action_goal)

    def waitForHand(self, prefix="right"):
        self.action_move_hand_client[prefix].wait_for_result()
        return self.action_move_hand_client[prefix].get_result()

    def hasContact(self, threshold, print_on_false=False):
        if self.T_F_C != None:
            return True
        return False

    def getKDLtf(self, base_frame, frame):
#        try:
        pose = self.listener.lookupTransform(base_frame, frame, rospy.Time(0))
#        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#            print "could not transform: ", base_frame, frame
#            return None
        return pm.fromTf(pose)

    fingers_links = {
        (0,0):'_HandFingerOneKnuckleOneLink',
        (0,1):'_HandFingerOneKnuckleTwoLink',
        (0,2):'_HandFingerOneKnuckleThreeLink',
        (1,0):'_HandFingerTwoKnuckleOneLink',
        (1,1):'_HandFingerTwoKnuckleTwoLink',
        (1,2):'_HandFingerTwoKnuckleThreeLink',
        (2,1):'_HandFingerThreeKnuckleTwoLink',
        (2,2):'_HandFingerThreeKnuckleThreeLink',
    }

    frames = {
        'Gr':'right_HandGripLink',
        'Pr':'right_HandPalmLink',
        'Fr00':'right_HandFingerOneKnuckleOneLink',
        'Fr01':'right_HandFingerOneKnuckleTwoLink',
        'Fr02':'right_HandFingerOneKnuckleThreeLink',
        'Fr10':'right_HandFingerTwoKnuckleOneLink',
        'Fr11':'right_HandFingerTwoKnuckleTwoLink',
        'Fr12':'right_HandFingerTwoKnuckleThreeLink',
        'Fr21':'right_HandFingerThreeKnuckleTwoLink',
        'Fr22':'right_HandFingerThreeKnuckleThreeLink',
    }

    def getTf(self, frame_from, frame_to):
        if frame_from in self.frames and frame_to in self.frames:
            return self.getKDLtf( self.frames[frame_from], self.frames[frame_to] )
        return None

