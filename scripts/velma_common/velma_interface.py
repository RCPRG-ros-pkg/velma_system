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
from barrett_hand_controller_msgs.msg import *
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

import velma_fk_ik

# reference frames:
# WO - world
# B - robot's base
# W - wrist
# Wr - right wrist
# Wl - left wrist
# G - grasp
# Gr - right grasp
# Gl - left grasp
# T - tool
# Tr - right tool
# Tl - left tool
# Frij - right hand finger i knuckle j

class VelmaInterface:
    """
Class used as Velma robot Interface.
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

    def getBodyJointLowerLimits(self):
        return self.body_joint_lower_limits

    def getBodyJointUpperLimits(self):
        return self.body_joint_upper_limits

    def getHeadJointLowerLimits(self):
        return self.head_joint_lower_limits

    def getHeadJointUpperLimits(self):
        return self.head_joint_upper_limits

    BEHAVOIUR_ERROR = 0
    BEHAVOIUR_OTHER = 1
    BEHAVOIUR_NONE = 2
    BEHAVOIUR_CART_IMP = 3
    BEHAVOIUR_CART_IMP_FT = 4
    BEHAVOIUR_JNT_IMP = 5
    BEHAVOIUR_CART_FCL = 6

    def getBehaviourName(self, behaviour_id):
        if not behaviour_id in self.behaviour_names:
            return "unknown behaviour"
        return self.behaviour_names[behaviour_id]

    def getControllerBehaviour(self):
        running = set()
        stopped = set()
        for item in self.conmanList().controller:
            if item.state == "stopped":
                stopped.add(item.name)
            elif item.state == "running":
                running.add(item.name)
            else:
                return self.BEHAVOIUR_ERROR
        for key in self.behavoiur_stopped_dict:
            if self.behavoiur_stopped_dict[key] == stopped:
                return key
        return self.BEHAVOIUR_OTHER
        
    def initConmanInterface(self):
        rospy.wait_for_service('/controller_manager/switch_controller')
        self.conmanSwitch = rospy.ServiceProxy('/controller_manager/switch_controller', controller_manager_msgs.srv.SwitchController)

        rospy.wait_for_service('/controller_manager/list_controllers')
        self.conmanList = rospy.ServiceProxy('/controller_manager/list_controllers', controller_manager_msgs.srv.ListControllers)

        self.behavoiur_stopped_dict = {self.BEHAVOIUR_ERROR:None,
            self.BEHAVOIUR_OTHER:None,
            self.BEHAVOIUR_NONE:set(['LeftForceControl', 'RightForceControl', 'RightForceTransformation', 'LeftForceTransformation', 'TrajectoryGeneratorJoint', 'VG', 'PoseIntLeft', 'JntLimit', 'CImp', 'HeadTrajectoryGeneratorJoint', 'PoseIntRight', 'JntImp']),
            self.BEHAVOIUR_CART_IMP:set(['LeftForceControl', 'RightForceControl', 'TrajectoryGeneratorJoint', 'VG', 'JntImp', 'RightForceTransformation', 'LeftForceTransformation']),
            self.BEHAVOIUR_CART_IMP_FT:set(['LeftForceControl', 'RightForceControl', 'TrajectoryGeneratorJoint', 'VG', 'JntImp']),
            self.BEHAVOIUR_JNT_IMP:set(['LeftForceControl', 'RightForceControl', 'RightForceTransformation', 'LeftForceTransformation', 'VG', 'CImp', 'PoseIntLeft', 'PoseIntRight']),
            self.BEHAVOIUR_CART_FCL:set(['TrajectoryGeneratorJoint', 'VG', 'JntImp', 'PoseIntLeft', 'PoseIntRight'])}

        self.possible_behavoiur_switches = set( [
            (self.BEHAVOIUR_OTHER,self.BEHAVOIUR_CART_IMP),
            (self.BEHAVOIUR_OTHER,self.BEHAVOIUR_JNT_IMP),
            (self.BEHAVOIUR_OTHER,self.BEHAVOIUR_CART_FCL),
            (self.BEHAVOIUR_CART_IMP,self.BEHAVOIUR_CART_IMP_FT),
            (self.BEHAVOIUR_CART_IMP,self.BEHAVOIUR_JNT_IMP),
            (self.BEHAVOIUR_CART_IMP,self.BEHAVOIUR_CART_FCL),
            (self.BEHAVOIUR_CART_IMP_FT,self.BEHAVOIUR_CART_IMP),
            (self.BEHAVOIUR_CART_IMP_FT,self.BEHAVOIUR_JNT_IMP),
            (self.BEHAVOIUR_JNT_IMP,self.BEHAVOIUR_CART_IMP),
            (self.BEHAVOIUR_JNT_IMP,self.BEHAVOIUR_CART_IMP_FT),
            (self.BEHAVOIUR_JNT_IMP,self.BEHAVOIUR_CART_FCL),
            (self.BEHAVOIUR_CART_FCL,self.BEHAVOIUR_CART_IMP),
            (self.BEHAVOIUR_CART_FCL,self.BEHAVOIUR_JNT_IMP), ] )

        self.behaviour_names = {
            self.BEHAVOIUR_ERROR:"BEHAVOIUR_ERROR",
            self.BEHAVOIUR_OTHER:"BEHAVOIUR_OTHER",
            self.BEHAVOIUR_NONE:"BEHAVOIUR_NONE",
            self.BEHAVOIUR_CART_IMP:"BEHAVOIUR_CART_IMP",
            self.BEHAVOIUR_CART_IMP_FT:"BEHAVOIUR_CART_IMP_FT",
            self.BEHAVOIUR_JNT_IMP:"BEHAVOIUR_JNT_IMP",
            self.BEHAVOIUR_CART_FCL:"BEHAVOIUR_CART_FCL", }

    def switchToBehavoiur(self, behaviour):
        current = self.getControllerBehaviour()
        if (not (current, behaviour) in self.possible_behavoiur_switches):
            print "VelmaInterface.switchToBehavoiur: could not switch behavoiur from " + self.getBehaviourName(current) + " to " + self.getBehaviourName(behaviour)
            return False

        if current == self.BEHAVOIUR_ERROR:
            return False
        elif current == self.BEHAVOIUR_OTHER:
            current_stopped = set()
            for item in self.conmanList().controller:
                if item.state == "stopped":
                    current_stopped.add(item.name)
        else:
            current_stopped = self.behavoiur_stopped_dict[current]
        des_stopped = self.behavoiur_stopped_dict[behaviour]

        cmd_start = current_stopped - des_stopped
        cmd_stop = des_stopped - current_stopped

        cmd_start_list = []
        for item in cmd_start:
            cmd_start_list.append(item)
        cmd_stop_list = []
        for item in cmd_stop:
            cmd_stop_list.append(item)

        if len(cmd_start_list) == 0 and len(cmd_stop_list) == 0:
            return True

        if self.conmanSwitch(cmd_start_list, cmd_stop_list, 2):
            return True

        return False

    def switchToJntImp(self):
        return self.switchToBehavoiur(self.BEHAVOIUR_JNT_IMP)

    def switchToCartImp(self):
        return self.switchToBehavoiur(self.BEHAVOIUR_CART_IMP)

    def switchToCartImpFT(self):
        return self.switchToBehavoiur(self.BEHAVOIUR_CART_IMP_FT)

    def switchToCartFcl(self):
        return self.switchToBehavoiur(self.BEHAVOIUR_CART_FCL)

    def isInCartImp(self):
        return self.getControllerBehaviour() == self.BEHAVOIUR_CART_IMP

    def isInCartImpFT(self):
        return self.getControllerBehaviour() == self.BEHAVOIUR_CART_IMP_FT

    def isInJntImp(self):
        return self.getControllerBehaviour() == self.BEHAVOIUR_JNT_IMP

    def __init__(self):

        # read the joint information from the ROS parameter server
        self.body_joint_names = rospy.get_param("/velma_controller/SplineTrajectoryActionJoint/joint_names")
        self.body_joint_lower_limits = rospy.get_param("/velma_controller/SplineTrajectoryActionJoint/lower_limits")
        self.body_joint_upper_limits = rospy.get_param("/velma_controller/SplineTrajectoryActionJoint/upper_limits")

        self.head_joint_names = rospy.get_param("/velma_controller/HeadSplineTrajectoryActionJoint/joint_names")
        self.head_joint_lower_limits = rospy.get_param("/velma_controller/HeadSplineTrajectoryActionJoint/lower_limits")
        self.head_joint_upper_limits = rospy.get_param("/velma_controller/HeadSplineTrajectoryActionJoint/upper_limits")

        self.all_joint_names = rospy.get_param("/velma_controller/JntPub/joint_names")

        self.js_pos = {}
        self.js_pos_history = []
        for i in range(200):
            self.js_pos_history.append( None )
        self.js_pos_history_idx = -1

        self.T_B_L = [PyKDL.Frame(),PyKDL.Frame(),PyKDL.Frame(),PyKDL.Frame(),PyKDL.Frame(),PyKDL.Frame(),PyKDL.Frame()]

        # parameters
        self.k_error = Wrench(Vector3(1.0, 1.0, 1.0), Vector3(0.5, 0.5, 0.5))
        self.T_B_W = None
        self.T_W_T = None
        self.T_Wl_El = None
        self.T_Wr_Er = None
        self.T_El_Wl = None
        self.T_Er_Wr = None
        self.current_max_wrench = Wrench(Vector3(20, 20, 20), Vector3(20, 20, 20))
        self.wrench_emergency_stop = False
        self.exit_on_emergency_stop = True

        self.last_contact_time = rospy.Time.now()

        self.joint_states_lock = Lock()

        # for score function
        self.failure_reason = "unknown"

        self.emergency_stop_active = False

        # cartesian wrist trajectory for right arm
        self.action_cart_traj_client = {
            'right':actionlib.SimpleActionClient("/right_arm/cartesian_trajectory", CartesianTrajectoryAction),
            'left':actionlib.SimpleActionClient("/left_arm/cartesian_trajectory", CartesianTrajectoryAction) }

        # joint trajectory for right arm
        self.action_right_joint_traj_client = None

        # cartesian tool trajectory for arms in the wrist frames
        self.action_tool_client = {
            'right':actionlib.SimpleActionClient("/right_arm/tool_trajectory", CartesianTrajectoryAction),
            'left':actionlib.SimpleActionClient("/left_arm/tool_trajectory", CartesianTrajectoryAction) }
        self.action_tool_client["right"].wait_for_server()
        self.action_tool_client["left"].wait_for_server()

        # cartesian impedance trajectory for right arm
        self.action_impedance_client = {
            'right':actionlib.SimpleActionClient("/right_arm/cartesian_impedance", CartesianImpedanceAction),
            'left':actionlib.SimpleActionClient("/left_arm/cartesian_impedance", CartesianImpedanceAction) }
        self.action_impedance_client["right"].wait_for_server()
        self.action_impedance_client["left"].wait_for_server()

        self.action_move_hand_client = {
            'right':actionlib.SimpleActionClient("/right_hand/move_hand", BHMoveAction),
            'left':actionlib.SimpleActionClient("/left_hand/move_hand", BHMoveAction) }
        self.action_move_hand_client["right"].wait_for_server()
        self.action_move_hand_client["left"].wait_for_server()


        self.pub_reset_left = rospy.Publisher("/left_hand/reset_fingers", std_msgs.msg.Empty, queue_size=100)
        self.pub_reset_right = rospy.Publisher("/right_hand/reset_fingers", std_msgs.msg.Empty, queue_size=100)

        self.listener = tf.TransformListener();
        self.br = tf.TransformBroadcaster()

        rospy.sleep(1.0)
        
        self.wrench_tab = []
        self.wrench_tab_index = 0
        self.wrench_tab_len = 4000
        for i in range(0,self.wrench_tab_len):
            self.wrench_tab.append( Wrench(Vector3(), Vector3()) )

        self.listener_joint_states = rospy.Subscriber('/joint_states', JointState, self.jointStatesCallback)

        subscribed_topics_list = [
            ('/right_arm/transformed_wrench', geometry_msgs.msg.Wrench),
            ('/left_arm/transformed_wrench', geometry_msgs.msg.Wrench),
            ('/right_arm/wrench', geometry_msgs.msg.Wrench),
            ('/left_arm/wrench', geometry_msgs.msg.Wrench),
            ('/right_arm/ft_sensor/wrench', geometry_msgs.msg.Wrench),
            ('/left_arm/ft_sensor/wrench', geometry_msgs.msg.Wrench), ]

        self.subscribed_topics = {}

        for topic in subscribed_topics_list:
            self.subscribed_topics[topic[0]] = [Lock(), None]
            sub = rospy.Subscriber(topic[0], topic[1], partial( self.topicCallback, topic = topic[0] ))
            self.subscribed_topics[topic[0]].append(sub)

        self.initConmanInterface()

    def topicCallback(self, data, topic):
        self.subscribed_topics[topic][0].acquire()
        self.subscribed_topics[topic][1] = copy.copy(data)
        self.subscribed_topics[topic][0].release()

    def getTopicData(self, topic):
        self.subscribed_topics[topic][0].acquire()
        data = copy.copy(self.subscribed_topics[topic][1])
        self.subscribed_topics[topic][0].release()
        return data

    def getRawFTr(self):
        return self.wrenchROStoKDL( self.getTopicData('/right_arm/ft_sensor/wrench') )

    def getRawFTl(self):
        return self.wrenchROStoKDL( self.getTopicData('/left_arm/ft_sensor/wrench') )

    def getTransformedFTr(self):
        return self.wrenchROStoKDL( self.getTopicData('/right_arm/transformed_wrench') )

    def getTransformedFTl(self):
        return self.wrenchROStoKDL( self.getTopicData('/left_arm/transformed_wrench') )

    def getWristWrenchr(self):
        return self.wrenchROStoKDL( self.getTopicData('/right_arm/wrench') )

    def getWristWrenchl(self):
        return self.wrenchROStoKDL( self.getTopicData('/left_arm/wrench') )

    def action_right_cart_traj_feedback_cb(self, feedback):
        self.action_right_cart_traj_feedback = copy.deepcopy(feedback)

    def wrenchKDLtoROS(self, wrKDL):
        return geometry_msgs.msg.Wrench(Vector3( wrKDL.force.x(), wrKDL.force.y(), wrKDL.force.z() ), Vector3( wrKDL.torque.x(), wrKDL.torque.y(), wrKDL.torque.z() ))

    def wrenchROStoKDL(self, wrROS):
        return PyKDL.Wrench( PyKDL.Vector(wrROS.force.x, wrROS.force.y, wrROS.force.z), PyKDL.Vector(wrROS.torque.x, wrROS.torque.y, wrROS.torque.z) )

    def moveEffector(self, prefix, T_B_Td, t, max_wrench, start_time=0.01, stamp=None, path_tol=None):
        behaviour = self.getControllerBehaviour()
        if behaviour != self.BEHAVOIUR_CART_IMP and behaviour != self.BEHAVOIUR_CART_IMP_FT:
            print "moveEffector " + prefix + ": wrong behaviour " + self.getBehaviourName(behaviour)
            return False

        self.joint_traj_active = False
        wrist_pose = pm.toMsg(T_B_Td)
#        self.br.sendTransform([wrist_pose.position.x, wrist_pose.position.y, wrist_pose.position.z], [wrist_pose.orientation.x, wrist_pose.orientation.y, wrist_pose.orientation.z, wrist_pose.orientation.w], rospy.Time.now(), "dest", "torso_base")

        action_trajectory_goal = CartesianTrajectoryGoal()
        if stamp != None:
            action_trajectory_goal.trajectory.header.stamp = stamp
        else:
            action_trajectory_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(start_time)
        action_trajectory_goal.trajectory.points.append(CartesianTrajectoryPoint(
        rospy.Duration(t),
        wrist_pose,
        Twist()))
        action_trajectory_goal.wrench_constraint = self.wrenchKDLtoROS(max_wrench)
        if path_tol != None:
            action_trajectory_goal.path_tolerance.position = geometry_msgs.msg.Vector3( path_tol.vel.x(), path_tol.vel.y(), path_tol.vel.z() )
            action_trajectory_goal.path_tolerance.rotation = geometry_msgs.msg.Vector3( path_tol.rot.x(), path_tol.rot.y(), path_tol.rot.z() )
        self.current_max_wrench = self.wrenchKDLtoROS(max_wrench)
        self.action_cart_traj_client[prefix].send_goal(action_trajectory_goal, feedback_cb = self.action_right_cart_traj_feedback_cb)
        return True

    def moveEffectorLeft(self, T_B_Tld, t, max_wrench, start_time=0.01, stamp=None, path_tol=None):
        self.moveEffector("left", T_B_Tld, t, max_wrench, start_time=start_time, stamp=stamp, path_tol=path_tol)

    def moveEffectorRight(self, T_B_Trd, t, max_wrench, start_time=0.01, stamp=None, path_tol=None):
        self.moveEffector("right", T_B_Trd, t, max_wrench, start_time=start_time, stamp=stamp, path_tol=path_tol)

    def moveEffectorTraj(self, prefix, list_T_B_Td, times, max_wrench, start_time=0.01, stamp=None):
        behaviour = self.getControllerBehaviour()
        if behaviour != self.BEHAVOIUR_CART_IMP and behaviour != self.BEHAVOIUR_CART_IMP_FT:
            print "moveEffector " + prefix + ": wrong behaviour " + self.getBehaviourName(bahaviour)
            return False

        action_trajectory_goal = CartesianTrajectoryGoal()
        if stamp != None:
            action_trajectory_goal.trajectory.header.stamp = stamp
        else:
            action_trajectory_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(start_time)

        i = 0
        for T_B_Td in list_T_B_Td:
            wrist_pose = pm.toMsg(T_B_Td)
            action_trajectory_goal.trajectory.points.append(CartesianTrajectoryPoint(
            rospy.Duration(times[i]),
            wrist_pose,
            Twist()))
            i += 1

        action_trajectory_goal.wrench_constraint = self.wrenchKDLtoROS(max_wrench)
        self.current_max_wrench = max_wrench
        self.action_cart_traj_client[prefix].send_goal(action_trajectory_goal)

    def moveEffectorTrajLeft(self, prefix, list_T_B_Tld, times, max_wrench, start_time=0.01, stamp=None):
        self.moveEffectorTraj("left", list_T_B_Tld, times, max_wrench, start_time=start_time, stamp=stamp)

    def moveEffectorTrajRight(self, prefix, list_T_B_Trd, times, max_wrench, start_time=0.01, stamp=None):
        self.moveEffectorTraj("right", list_T_B_Trd, times, max_wrench, start_time=start_time, stamp=stamp)

    cartesian_trajectory_result_names = {
        CartesianTrajectoryResult.SUCCESSFUL:'SUCCESSFUL',
        CartesianTrajectoryResult.INVALID_GOAL:'INVALID_GOAL',
        CartesianTrajectoryResult.OLD_HEADER_TIMESTAMP:'OLD_HEADER_TIMESTAMP',
        CartesianTrajectoryResult.PATH_TOLERANCE_VIOLATED:'PATH_TOLERANCE_VIOLATED',
        CartesianTrajectoryResult.GOAL_TOLERANCE_VIOLATED:'GOAL_TOLERANCE_VIOLATED', }

    def waitForEffector(self, prefix, timeout_s=None):
        if timeout_s == None:
            timeout_s = 0
        if not self.action_cart_traj_client[prefix].wait_for_result(timeout=rospy.Duration(timeout_s)):
            return None
        result = self.action_cart_traj_client[prefix].get_result()
        if result.error_code != 0:
            print "waitForEffector(" + prefix + "): action failed with error_code=" + str(result.error_code) + " (" + self.cartesian_trajectory_result_names[result.error_code] + ")"
        return result.error_code

    def waitForEffectorLeft(self, timeout_s=None):
        return self.waitForEffector("left", timeout_s=timeout_s)

    def waitForEffectorRight(self, timeout_s=None):
        return self.waitForEffector("right", timeout_s=timeout_s)

#    def isActiveEffector(self, prefix):
#        self.action_cart_traj_client[prefix].get_state()
#        result = self.action_cart_traj_client[prefix].get_result()
#        if result.error_code != 0:
#            print "waitForEffector(" + prefix + "): action failed with error_code=" + str(result.error_code) + " (" + self.cartesian_trajectory_result_names[result.error_code] + ")"
#        return result.error_code

#    def isActiveEffectorRight(self):
#        return self.isActiveEffector("right")

#    def isActiveEffectorLeft(self):
#        return self.isActiveEffector("left")

    def moveTool(self, prefix, T_W_T, t, stamp=None):
        ros_T_W_T = pm.toMsg(T_W_T)

        action_tool_goal = CartesianTrajectoryGoal()
        if stamp != None:
            action_tool_goal.trajectory.header.stamp = stamp
        else:
            action_tool_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.01)
        action_tool_goal.trajectory.points.append(CartesianTrajectoryPoint(
        rospy.Duration(t),
        ros_T_W_T,
        Twist()))
        self.action_tool_client[prefix].send_goal(action_tool_goal)

    def moveToolLeft(self, T_Wl_Tl, t, stamp=None):
        self.moveTool("left", T_Wl_Tl, t, stamp=stamp)

    def moveToolRight(self, T_Wr_Tr, t, stamp=None):
        self.moveTool("right", T_Wr_Tr, t, stamp=stamp)

    def waitForTool(self, prefix):
        self.action_tool_client[prefix].wait_for_result()
        result = self.action_tool_client[prefix].get_result()
        if result.error_code != 0:
            print "waitForTool(" + prefix + "): action failed with error_code=" + str(result.error_code)
        return result.error_code

    def waitForToolLeft(self):
        return self.waitForTool("left")

    def waitForToolRight(self):
        return self.waitForTool("right")

    def moveImpedance(self, prefix, k, t, stamp=None, damping=Wrench(Vector3(0.7, 0.7, 0.7),Vector3(0.7, 0.7, 0.7))):
        action_impedance_goal = CartesianImpedanceGoal()
        if stamp != None:
            action_impedance_goal.trajectory.header.stamp = stamp
        else:
            action_impedance_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
        action_impedance_goal.trajectory.points.append(CartesianImpedanceTrajectoryPoint(
        rospy.Duration(t),
        CartesianImpedance(self.wrenchKDLtoROS(k), damping)))
        self.action_impedance_client[prefix].send_goal(action_impedance_goal)

    def moveImpedanceLeft(self, k, t, stamp=None, damping=Wrench(Vector3(0.7, 0.7, 0.7),Vector3(0.7, 0.7, 0.7))):
        self.moveImpedance("left", k, t, stamp=stamp, damping=damping)

    def moveImpedanceRight(self, k, t, stamp=None, damping=Wrench(Vector3(0.7, 0.7, 0.7),Vector3(0.7, 0.7, 0.7))):
        self.moveImpedance("right", k, t, stamp=stamp, damping=damping)

    def moveImpedanceTraj(self, prefix, k_n, t_n, stamp=None, damping=Wrench(Vector3(0.7, 0.7, 0.7),Vector3(0.7, 0.7, 0.7))):
        action_impedance_goal = CartesianImpedanceGoal()
        if stamp != None:
            action_impedance_goal.trajectory.header.stamp = stamp
        else:
            action_impedance_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
        i = 0
        for k in k_n:
            action_impedance_goal.trajectory.points.append(CartesianImpedanceTrajectoryPoint(
            rospy.Duration(t_n[i]),
            CartesianImpedance(self.wrenchKDLtoROS(k), damping)))
        self.action_impedance_client[prefix].send_goal(action_impedance_goal)

    def moveImpedanceTrajLeft(self, k_n, t_n, stamp=None, damping=Wrench(Vector3(0.7, 0.7, 0.7),Vector3(0.7, 0.7, 0.7))):
        self.moveImpedanceTraj(self, "left", k_n, t_n, stamp=stamp, damping=damping)

    def moveImpedanceTrajRight(self, k_n, t_n, stamp=None, damping=Wrench(Vector3(0.7, 0.7, 0.7),Vector3(0.7, 0.7, 0.7))):
        self.moveImpedanceTraj(self, "right", k_n, t_n, stamp=stamp, damping=damping)

    def waitForImpedance(self, prefix):
        self.action_impedance_client[prefix].wait_for_result()
        result = self.action_impedance_client[prefix].get_result()
        if result.error_code != 0:
            print "waitForImpedance(" + prefix + "): action failed with error_code=" + str(result.error_code)
        return result.error_code

    def waitForImpedanceLeft(self):
        return self.waitForImpedance("left")

    def waitForImpedanceRight(self):
        return self.waitForImpedance("right")

    def moveJoint(self, q_dest, joint_names, time, start_time=0.2):
        if not (not self.cartesian_impedance_active and self.joint_impedance_active):
            print "FATAL ERROR: moveJoint"
            exit(0)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.body_joint_names

        vel = []
        q_dest_all = []
        for joint_name in self.body_joint_names:
            if joint_name in joint_names:
                q_idx = joint_names.index(joint_name)
                q_dest_all.append(q_dest[q_idx])
                vel.append(0)
            else:
                q_dest_all.append(self.js_pos[joint_name])
                vel.append(0)

        goal.trajectory.points.append(JointTrajectoryPoint(q_dest_all, vel, [], [], rospy.Duration(time)))
        position_tol = 5.0/180.0 * math.pi
        velocity_tol = 5.0/180.0 * math.pi
        acceleration_tol = 1.0/180.0 * math.pi
        for joint_name in goal.trajectory.joint_names:
            goal.path_tolerance.append(JointTolerance(joint_name, position_tol, velocity_tol, acceleration_tol))
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(start_time)
        self.joint_traj_active = True
        self.action_right_joint_traj_client.send_goal(goal)

    def moveJointTraj(self, traj, joint_names, start_time=0.2):
        if not (not self.cartesian_impedance_active and self.joint_impedance_active):
            print "FATAL ERROR: moveJointTraj"
            exit(0)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.body_joint_names

        pos = traj[0]
        vel = traj[1]
        acc = traj[2]
        dti = traj[3]
        time = 0.0

        for node_idx in range(0, len(pos)):
            time += dti[node_idx]
            q_dest_all = []
            vel_dest_all = []

            for joint_name in self.body_joint_names:
                if joint_name in joint_names:
                    q_idx = joint_names.index(joint_name)
                    q_dest_all.append(pos[node_idx][q_idx])
                    if vel != None:
                        vel_dest_all.append(vel[node_idx][q_idx])
                    else:
                        vel_dest_all.append(0)
                else:
                    q_dest_all.append(self.js_pos[joint_name])
                    vel_dest_all.append(0)

            goal.trajectory.points.append(JointTrajectoryPoint(q_dest_all, vel_dest_all, [], [], rospy.Duration(time)))

        position_tol = 5.0/180.0 * math.pi
        velocity_tol = 5.0/180.0 * math.pi
        acceleration_tol = 5.0/180.0 * math.pi
        for joint_name in goal.trajectory.joint_names:
            goal.path_tolerance.append(JointTolerance(joint_name, position_tol, velocity_tol, acceleration_tol))
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(start_time)
        self.joint_traj_active = True
        self.action_right_joint_traj_client.send_goal(goal)

    def waitForJoint(self):
        self.action_right_joint_traj_client.wait_for_result()
        return self.action_right_joint_traj_client.get_result()

    def stopArm(self, prefix):
        try:
            self.action_cart_traj_client[prefix].cancel_goal()
        except:
            pass
        try:
            self.action_tool_client[prefix].cancel_goal()
        except:
            pass

    def stopArmLeft(self):
        self.stopArm("left")

    def stopArmRight(self):
        self.stopArm("right")

    def emergencyStop(self):
        self.moveImpedance(self.k_error, 0.5)
        self.stopArmLeft()
        self.stopArmRight()
        self.emergency_stop_active = True
        print "emergency stop"

    def checkStopCondition(self, t=0.0):

        end_t = rospy.Time.now()+rospy.Duration(t+0.0001)
        while rospy.Time.now()<end_t:
            if rospy.is_shutdown():
                self.emergencyStop()
                print "emergency stop: interrupted  %s  %s"%(self.getMaxWrench(), self.wrench_tab_index)
                self.failure_reason = "user_interrupt"
                rospy.sleep(1.0)
            if self.wrench_emergency_stop:
                self.emergencyStop()
                print "too big wrench"
                self.failure_reason = "too_big_wrench"
                rospy.sleep(1.0)

#            if (self.action_cart_traj_client != None) and (self.action_cart_traj_client.gh) and ((self.action_cart_traj_client.get_state()==GoalStatus.REJECTED) or (self.action_cart_traj_client.get_state()==GoalStatus.ABORTED)):
#                state = self.action_cart_traj_client.get_state()
#                result = self.action_cart_traj_client.get_result()
#                self.emergencyStop()
#                print "emergency stop: traj_err: %s ; %s ; max_wrench: %s   %s"%(state, result, self.getMaxWrench(), self.wrench_tab_index)
#                self.failure_reason = "too_big_wrench_trajectory"
#                rospy.sleep(1.0)

#            if (self.action_tool_client.gh) and ((self.action_tool_client.get_state()==GoalStatus.REJECTED) or (self.action_tool_client.get_state()==GoalStatus.ABORTED)):
#                state = self.action_tool_client.get_state()
#                result = self.action_tool_client.get_result()
#                self.emergencyStop()
#                print "emergency stop: tool_err: %s ; %s ; max_wrench: %s   %s"%(state, result, self.getMaxWrench(), self.wrench_tab_index)
#                self.failure_reason = "too_big_wrench_tool"
#                rospy.sleep(1.0)
            rospy.sleep(0.01)
        return self.emergency_stop_active

    # ex.
    # q = (120.0/180.0*numpy.pi, 120.0/180.0*numpy.pi, 40.0/180.0*numpy.pi, 180.0/180.0*numpy.pi)
    # robot.move_hand_client("right", q)
#    def move_hand_client(self, q, v=(1.2, 1.2, 1.2, 1.2), t=(2000.0, 2000.0, 2000.0, 2000.0)):
#        rospy.wait_for_service('/' + self.prefix + '_hand/move_hand')
#        try:
#            move_hand = rospy.ServiceProxy('/' + self.prefix + '_hand/move_hand', BHMoveHand)
#            resp1 = move_hand(q[0], q[1], q[2], q[3], v[0], v[1], v[2], v[3], t[0], t[1], t[2], t[3])
#        except rospy.ServiceException, e:
#            print "Service call failed: %s"%e

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
        self.action_move_hand_client[prefix].send_goal(action_goal)

    def moveHandLeft(self, q, v, t, maxPressure, hold=False):
        self.moveHand(q, v, t, maxPressure, hold=hold, prefix="left")

    def moveHandRight(self, q, v, t, maxPressure, hold=False):
        self.moveHand(q, v, t, maxPressure, hold=hold, prefix="right")

    def waitForHand(self, prefix="right"):
        self.action_move_hand_client[prefix].wait_for_result()
        return self.action_move_hand_client[prefix].get_result()

    def waitForHandLeft(self):
        return self.waitForHand(prefix="left")

    def waitForHandRight(self):
        return self.waitForHand(prefix="right")

    def hasContact(self, threshold, print_on_false=False):
        if self.T_F_C != None:
            return True
        return False

    def getKDLtf(self, base_frame, frame):
        pose = self.listener.lookupTransform(base_frame, frame, rospy.Time(0))
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
        'Wo':'world',
        'B':'torso_base',
        'Wr':'right_arm_7_link',
        'Wl':'left_arm_7_link',
        'Gr':'right_HandGripLink',
        'Gl':'left_HandGripLink',
        'Tr':'right_arm_tool',
        'Tl':'left_arm_tool',
        'Fr00':'right_HandFingerOneKnuckleOneLink',
        'Fr01':'right_HandFingerOneKnuckleTwoLink',
        'Fr02':'right_HandFingerOneKnuckleThreeLink',
        'Fr10':'right_HandFingerTwoKnuckleOneLink',
        'Fr11':'right_HandFingerTwoKnuckleTwoLink',
        'Fr12':'right_HandFingerTwoKnuckleThreeLink',
        'Fr21':'right_HandFingerThreeKnuckleTwoLink',
        'Fr22':'right_HandFingerThreeKnuckleThreeLink',
        'Fl00':'left_HandFingerOneKnuckleOneLink',
        'Fl01':'left_HandFingerOneKnuckleTwoLink',
        'Fl02':'left_HandFingerOneKnuckleThreeLink',
        'Fl10':'left_HandFingerTwoKnuckleOneLink',
        'Fl11':'left_HandFingerTwoKnuckleTwoLink',
        'Fl12':'left_HandFingerTwoKnuckleThreeLink',
        'Fl21':'left_HandFingerThreeKnuckleTwoLink',
        'Fl22':'left_HandFingerThreeKnuckleThreeLink',
    }

    def getTf(self, frame_from, frame_to):
        if frame_from in self.frames and frame_to in self.frames:
            return self.getKDLtf( self.frames[frame_from], self.frames[frame_to] )
        return None

    def handleEmergencyStop(self):
        if self.emergency_stop_active:
            ch = '_'
            while (ch != 'e') and (ch != 'n') and (ch != 'r'):
                ch = raw_input("Emergency stop active... (e)xit, (n)ext case, (r)epeat case: ")
            if ch == 'e':
                exit(0)
            if ch == 'n':
                self.action = "next"
                self.index += 1
            if ch == 'r':
                self.action = "repeat"
            self.updateTransformations()
            print "moving desired pose to current pose"
            self.emergency_stop_active = False
            self.moveWrist(self.T_B_W, 2.0, Wrench(Vector3(25,25,25), Vector3(5,5,5)))
            self.checkStopCondition(2.0)
            return True
        return False

    def getMovementTime(self, T_B_Wd, max_v_l = 0.1, max_v_r = 0.2):
        self.updateTransformations()
        twist = PyKDL.diff(self.T_B_W, T_B_Wd, 1.0)
        v_l = twist.vel.Norm()
        v_r = twist.rot.Norm()
#        print "v_l: %s   v_r: %s"%(v_l, v_r)
        f_v_l = v_l/max_v_l
        f_v_r = v_r/max_v_r
        if f_v_l > f_v_r:
            duration = f_v_l
        else:
            duration = f_v_r
        if duration < 0.2:
            duration = 0.2
        return duration

    def getMovementTime2(self, T_B_Wd1, T_B_Wd2, max_v_l = 0.1, max_v_r = 0.2):
        twist = PyKDL.diff(T_B_Wd1, T_B_Wd2, 1.0)
        v_l = twist.vel.Norm()
        v_r = twist.rot.Norm()
        print "v_l: %s   v_r: %s"%(v_l, v_r)
        f_v_l = v_l/max_v_l
        f_v_r = v_r/max_v_r
        if f_v_l > f_v_r:
            duration = f_v_l
        else:
            duration = f_v_r
        if duration < 0.5:
            duration = 0.5
        return duration

