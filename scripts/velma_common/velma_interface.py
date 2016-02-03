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

import tf
from tf import *
from tf.transformations import * 
import tf_conversions.posemath as pm
from tf2_msgs.msg import *
from controller_manager_msgs.srv import *


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

class VelmaInterface:
    """
Class used as Velma robot Interface.
"""

#    def getJointStatesByNames(self, dof_names):
#        js = []
#        for joint_name in dof_names:
#            js.append( self.js_pos[joint_name] )
#        return js

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

#    def getMaxWrench(self):
#        wrench = Wrench()
#        for i in range(0,self.wrench_tab_len):
#            if abs(self.wrench_tab[i].force.x) > wrench.force.x:
#                wrench.force.x = abs(self.wrench_tab[i].force.x)
#            if abs(self.wrench_tab[i].force.y) > wrench.force.y:
#                wrench.force.y = abs(self.wrench_tab[i].force.y)
#            if abs(self.wrench_tab[i].force.z) > wrench.force.z:
#                wrench.force.z = abs(self.wrench_tab[i].force.z)
#            if abs(self.wrench_tab[i].torque.x) > wrench.torque.x:
#                wrench.torque.x = abs(self.wrench_tab[i].torque.x)
#            if abs(self.wrench_tab[i].torque.y) > wrench.torque.y:
#                wrench.torque.y = abs(self.wrench_tab[i].torque.y)
#            if abs(self.wrench_tab[i].torque.z) > wrench.torque.z:
#                wrench.torque.z = abs(self.wrench_tab[i].torque.z)
#        return wrench

#    def wrenchCallback(self, wrench):
#        self.wrench_tab[self.wrench_tab_index] = wrench
#        self.wrench_tab_index += 1
#        if self.wrench_tab_index >= self.wrench_tab_len:
#            self.wrench_tab_index = 0
#        wfx = abs(wrench.force.x)
#        wfy = abs(wrench.force.y)
#        wfz = abs(wrench.force.z)
#        wtx = abs(wrench.torque.x)
#        wty = abs(wrench.torque.y)
#        wtz = abs(wrench.torque.z)
#        if not self.joint_traj_active:
#            if (wfx>self.current_max_wrench.force.x*2.0) or (wfy>self.current_max_wrench.force.y*2.0) or (wfz>self.current_max_wrench.force.z*2.0) or (wtx>self.current_max_wrench.torque.x*2.0) or (wty>self.current_max_wrench.torque.y*2.0) or (wtz>self.current_max_wrench.torque.z*2.0):
#                self.wrench_emergency_stop = True

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

        self.joint_impedance_active = False
        self.cartesian_impedance_active = False
        self.joint_traj_active = False

        self.T_B_L = [PyKDL.Frame(),PyKDL.Frame(),PyKDL.Frame(),PyKDL.Frame(),PyKDL.Frame(),PyKDL.Frame(),PyKDL.Frame()]

        # parameters
#        self.prefix="right"
        self.k_error = Wrench(Vector3(1.0, 1.0, 1.0), Vector3(0.5, 0.5, 0.5))
        self.T_B_W = None
        self.T_W_T = None #PyKDL.Frame(PyKDL.Vector(0.2,-0.05,0))    # tool transformation
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
        self.action_cart_traj_client = {}

        # joint trajectory for right arm
        self.action_right_joint_traj_client = None

        # cartesian tool trajectory for arms in the wrist frames
        self.action_tool_client = {}
        self.action_tool_client["right"] = actionlib.SimpleActionClient("/right_arm/tool_trajectory", CartesianTrajectoryAction)
        self.action_tool_client["right"].wait_for_server()
        self.action_tool_client["left"] = actionlib.SimpleActionClient("/left_arm/tool_trajectory", CartesianTrajectoryAction)
        self.action_tool_client["left"].wait_for_server()

        # cartesian impedance trajectory for right arm
        self.action_impedance_client = {}
        self.action_impedance_client["right"] = actionlib.SimpleActionClient("/right_arm/cartesian_impedance", CartesianImpedanceAction)
        self.action_impedance_client["right"].wait_for_server()
        self.action_impedance_client["left"] = actionlib.SimpleActionClient("/left_arm/cartesian_impedance", CartesianImpedanceAction)
        self.action_impedance_client["left"].wait_for_server()

        self.action_move_hand_client = {}
        self.action_move_hand_client["right"] = actionlib.SimpleActionClient("/right_hand/move_hand", BHMoveAction)
        self.action_move_hand_client["right"].wait_for_server()

        self.action_move_hand_client["left"] = actionlib.SimpleActionClient("/left_hand/move_hand", BHMoveAction)
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

#        rospy.Subscriber('/'+self.prefix+'_arm/wrench', Wrench, self.wrenchCallback)
        joint_states_listener = rospy.Subscriber('/joint_states', JointState, self.jointStatesCallback)

    def action_right_cart_traj_feedback_cb(self, feedback):
        self.action_right_cart_traj_feedback = copy.deepcopy(feedback)

    def wrenchKDLtoROS(self, wrKDL):
        return geometry_msgs.msg.Wrench(Vector3( wrKDL.force.x(), wrKDL.force.y(), wrKDL.force.z() ), Vector3( wrKDL.torque.x(), wrKDL.torque.y(), wrKDL.torque.z() ))

    def moveEffector(self, prefix, T_B_Td, t, max_wrench, start_time=0.01, stamp=None, path_tol=None):
        if not (self.cartesian_impedance_active and not self.joint_impedance_active):
            print "FATAL ERROR: moveEffector"
            exit(0)

        self.joint_traj_active = False
        wrist_pose = pm.toMsg(T_B_Td)
        self.br.sendTransform([wrist_pose.position.x, wrist_pose.position.y, wrist_pose.position.z], [wrist_pose.orientation.x, wrist_pose.orientation.y, wrist_pose.orientation.z, wrist_pose.orientation.w], rospy.Time.now(), "dest", "torso_base")

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

    def moveEffectorLeft(self, T_B_Tld, t, max_wrench, start_time=0.01, stamp=None, path_tol=None):
        self.moveEffector("left", T_B_Tld, t, max_wrench, start_time=start_time, stamp=stamp, path_tol=path_tol)

    def moveEffectorRight(self, T_B_Trd, t, max_wrench, start_time=0.01, stamp=None, path_tol=None):
        self.moveEffector("right", T_B_Trd, t, max_wrench, start_time=start_time, stamp=stamp, path_tol=path_tol)

    def moveEffectorTraj(self, prefix, list_T_B_Td, times, max_wrench, start_time=0.01, stamp=None):
        if not (self.cartesian_impedance_active and not self.joint_impedance_active):
            print "FATAL ERROR: moveEffectorTraj"
            exit(0)

        self.joint_traj_active = False

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

    def waitForEffector(self, prefix):
        self.action_cart_traj_client[prefix].wait_for_result()
        result = self.action_cart_traj_client[prefix].get_result()
        if result.error_code != 0:
            print "waitForEffector(" + prefix + "): action failed with error_code=" + str(result.error_code)
        return result.error_code

    def waitForEffectorLeft(self):
        return self.waitForEffector("left")

    def waitForEffectorRight(self):
        return self.waitForEffector("right")

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

    def updateTransformations(self):
        self.T_B_Wr = self.getKDLtf('torso_base', 'right_arm_7_link')
        self.T_B_Wl = self.getKDLtf('torso_base', 'left_arm_7_link')

        self.T_Wr_Tr = self.getKDLtf('right_arm_7_link', 'right_arm_tool')
        self.T_Wl_Tl = self.getKDLtf('left_arm_7_link', 'left_arm_tool')

        self.T_Wr_Gr = self.getKDLtf('right_arm_7_link', 'right_HandGripLink')
        self.T_Wl_Gl = self.getKDLtf('left_arm_7_link', 'left_HandGripLink')

#        for i in range(0,7):
#            pose = self.listener.lookupTransform('torso_base', self.prefix+'_arm_' + str(i+1) + '_link', rospy.Time(0))
#            self.T_B_L[i] = pm.fromTf(pose)

#        pose = self.listener.lookupTransform('/'+self.prefix+'_HandPalmLink', '/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', rospy.Time(0))
#        self.T_E_F = pm.fromTf(pose)
#        self.T_F_E = self.T_E_F.Inverse()

#        pose = self.listener.lookupTransform('/'+self.prefix+'_HandPalmLink', '/'+self.prefix+'_HandFingerOneKnuckleThreeLink', rospy.Time(0))
#        self.T_E_F13 = pm.fromTf(pose)
#        self.T_F13_E = self.T_E_F13.Inverse()

#        pose = self.listener.lookupTransform('/'+self.prefix+'_HandPalmLink', '/'+self.prefix+'_HandFingerTwoKnuckleThreeLink', rospy.Time(0))
#        self.T_E_F23 = pm.fromTf(pose)
#        self.T_F23_E = self.T_E_F23.Inverse()

#        pose = self.listener.lookupTransform('/'+self.prefix+'_HandPalmLink', '/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', rospy.Time(0))
#        self.T_E_F33 = pm.fromTf(pose)
#        self.T_F33_E = self.T_E_F33.Inverse()

#        if self.T_Wr_Er == None:
#            pose = self.listener.lookupTransform('right_arm_7_link', 'right_HandPalmLink', rospy.Time(0))
#            self.T_Wr_Er = pm.fromTf(pose)
#            self.T_Er_Wr = self.T_Wr_Er.Inverse()

#        if self.T_Wl_El == None:
#            pose = self.listener.lookupTransform('left_arm_7_link', 'left_HandPalmLink', rospy.Time(0))
#            self.T_Wl_El = pm.fromTf(pose)
#            self.T_El_Wl = self.T_Wl_El.Inverse()

#        pose = self.listener.lookupTransform('torso_base', self.prefix+'_arm_cmd', rospy.Time(0))
#        self.T_B_T_cmd = pm.fromTf(pose)

#        pose = self.listener.lookupTransform('right_HandPalmLink', 'right_HandFingerOneKnuckleOneLink', rospy.Time(0))
#        self.T_E_F11 = pm.fromTf(pose)

#        pose = self.listener.lookupTransform('right_HandPalmLink', 'right_HandFingerTwoKnuckleOneLink', rospy.Time(0))
#        self.T_E_F21 = pm.fromTf(pose)

#        self.T_E_F31 = PyKDL.Frame()

#        pose = self.listener.lookupTransform('torso_base', 'camera', rospy.Time(0))
#        self.T_B_C = pm.fromTf(pose)

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

#    def updateAndMoveTool(self, tool, duration):
#        self.T_W_T = copy.deepcopy(tool)    # tool transformation
#        self.updateTransformations()
#        print "setting the tool to %s relative to wrist frame"%(self.T_W_T)
#        # move both tool position and wrist position - the gripper holds its position
#        print "moving wrist"
#        # we assume that during the initialization there are no contact forces, so we limit the wrench
#        stamp = rospy.Time.now() + rospy.Duration(1.0)
#        self.moveWrist( self.T_B_W, duration, Wrench(Vector3(10, 10, 10), Vector3(2, 2, 2)), stamp=stamp)
#        self.moveTool( self.T_W_T, duration, stamp=stamp )

#    def updateAndMoveToolOnly(self, tool, duration):
#        self.T_W_T = copy.deepcopy(tool)    # tool transformation
#        self.updateTransformations()
#        print "setting the tool to %s relative to wrist frame"%(self.T_W_T)
#        # move both tool position and wrist position - the gripper holds its position
#        print "moving tool"
#        self.moveTool( self.T_W_T, duration )

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

    def switchToJoint(self):
        self.cartesian_impedance_active = False
        result = False
        try:
            if not hasattr(self, 'conmanSwitch') or self.conmanSwitch == None:
                rospy.wait_for_service('/controller_manager/switch_controller')
                self.conmanSwitch = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            # '2' is for STRICT
            if self.conmanSwitch(['JntImp', 'TrajectoryGeneratorJoint'], ['CImp', 'PoseIntLeft', 'PoseIntRight'], 2):
                # joint trajectory for right arm
                if self.action_right_joint_traj_client == None:
                    self.action_right_joint_traj_client = actionlib.SimpleActionClient('/spline_trajectory_action_joint', FollowJointTrajectoryAction)
                    self.action_right_joint_traj_client.wait_for_server()
                result = True
        except rospy.ROSInterruptException:
            print "rospy.ROSInterruptException"
        except IOError:
            print "IOError"
        except KeyError:
            print "KeyError"
        if not result:
            print "FATAL ERROR: switchToJoint"
            exit(0)
        self.joint_impedance_active = True
        return result

    def switchToCart(self):
        self.joint_impedance_active = False
        result = False
        try:
            if not hasattr(self, 'conmanSwitch') or self.conmanSwitch == None:
                rospy.wait_for_service('/controller_manager/switch_controller')
                self.conmanSwitch = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            # '2' is for STRICT
            if self.conmanSwitch(['CImp', 'PoseIntLeft', 'PoseIntRight'], ['JntImp', 'TrajectoryGeneratorJoint'], 2):
                # cartesian wrist trajectory for right arm
                if len(self.action_cart_traj_client) == 0:
                    self.action_cart_traj_client["left"] = actionlib.SimpleActionClient("/left_arm/cartesian_trajectory", CartesianTrajectoryAction)
                    self.action_cart_traj_client["right"] = actionlib.SimpleActionClient("/right_arm/cartesian_trajectory", CartesianTrajectoryAction)
                    self.action_cart_traj_client["left"].wait_for_server()
                    self.action_cart_traj_client["right"].wait_for_server()
                result = True

        except rospy.ROSInterruptException:
            print "rospy.ROSInterruptException"
        except IOError:
            print "IOError"
        except KeyError:
            print "KeyError"
        if not result:
            print "FATAL ERROR: switchToCart"
            exit(0)
        self.cartesian_impedance_active = True
        return result

    def isJointImpedanceActive(self):
        if self.joint_impedance_active and not self.cartesian_impedance_active:
            return True
        return False

    def isCartesianImpedanceActive(self):
        if not self.joint_impedance_active and self.cartesian_impedance_active:
            return True
        return False


