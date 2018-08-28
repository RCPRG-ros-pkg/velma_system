## This file contains ROS-based Python interface for WUT Velma Robot and some helper functions.
# @file velma_interface.py
# @ingroup python_api

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
import math
import time
import copy
import threading
from functools import partial
import PyKDL

from geometry_msgs.msg import Wrench, Vector3, Twist
from sensor_msgs.msg import JointState
from barrett_hand_msgs.msg import *
from barrett_hand_action_msgs.msg import *
from cartesian_trajectory_msgs.msg import *
from trapezoid_trajectory_msgs.msg import TrapezoidTrajectoryAction, TrapezoidTrajectoryResult, TrapezoidTrajectoryGoal
from motor_action_msgs.msg import *
from behavior_switch_action_msgs.msg import BehaviorSwitchAction, BehaviorSwitchGoal
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult, FollowJointTrajectoryGoal, JointTolerance
import diagnostic_msgs.msg
import tf_conversions.posemath as pm

import subsystem_common

import xml.dom.minidom as minidom

def isConfigurationClose(q_map1, q_map2, tolerance=0.1):
    """!
    Check if two configurations of robot body are close within tolerance.

    @param q_map1 dictionary: Dictionary {name:position} that maps joint names to their positions.
    @param q_map2 dictionary: Dictionary {name:position} that maps joint names to their positions.

    @return True if two configurations are close within tolerance (for every joint), False otherwise.

    @exception KeyError: Either q_map1 or q_map2 dictionaries does not contain required key, i.e. one
        of robot body joint names.
    """
    joint_names = ['torso_0_joint', 'right_arm_0_joint', 'right_arm_1_joint',
        'right_arm_2_joint', 'right_arm_3_joint', 'right_arm_4_joint', 'right_arm_5_joint',
        'right_arm_6_joint', 'left_arm_0_joint', 'left_arm_1_joint', 'left_arm_2_joint',
        'left_arm_3_joint', 'left_arm_4_joint', 'left_arm_5_joint', 'left_arm_6_joint']

    for joint_name in joint_names:
        if abs(q_map1[joint_name] - q_map2[joint_name]) > tolerance:
            return False
    return True

def isHeadConfigurationClose(q1, q2, tolerance=0.1):
    """!
    Check if two configurations of robot neck are close within tolerance.

    @param q1 tuple: 2-tuple of neck joints positions.
    @param q2 tuple: 2-tuple of neck joints positions.

    @return True if two configurations are close within tolerance (for every joint), False otherwise.
    """
    return abs(q1[0]-q2[0]) < tolerance and abs(q1[1]-q2[1]) < tolerance

def isHandConfigurationClose(current_q, dest_q, tolerance=0.1):
    """!
    Check if two configurations of robot hand are close within tolerance.

    @param current_q tuple: 8-tuple of hand joints positions.
    @param dest_q tuple: 4-tuple of hand DOFs positions.

    @return True if two configurations are close within tolerance (for every joint), False otherwise.
    """
    return abs(current_q[0]-dest_q[3]) < tolerance and\
        abs(current_q[1]-dest_q[0]) < tolerance and\
        abs(current_q[4]-dest_q[1]) < tolerance and\
        abs(current_q[6]-dest_q[2]) < tolerance

class VelmaInterface:
    """!
    ROS-based, Python interface class for WUT Velma Robot.

    This class implements methods that use ROS topics, actions and services
    to communicate with velma_task_cs_ros_interface subsystem of velma_task
    agent.
    """

    #
    # Classes
    #
    class VisualMesh:
        """!
        This class contains information about geometric object: mesh.
        """
        def __init__(self):
            self.type = "mesh"
            self.origin = None
            self.filename = None

    class Link:
        """!
        This class contains information about single link of robot.
        """
        def __init__(self):
            self.name = None
            self.visuals = []

    #
    # Private data
    #
    _frames = {
        'Wo':'world',
        'B':'torso_base',
        'Wr':'right_arm_7_link',
        'Wl':'left_arm_7_link',
        'Wright':'right_arm_7_link',
        'Wleft':'left_arm_7_link',
        'Gr':'right_HandGripLink',
        'Gl':'left_HandGripLink',
        'Tr':'right_arm_tool',
        'Tl':'left_arm_tool',
        'Tright':'right_arm_tool',
        'Tleft':'left_arm_tool',
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

    _cartesian_trajectory_result_names = {
        CartImpResult.SUCCESSFUL:'SUCCESSFUL',
        CartImpResult.INVALID_GOAL:'INVALID_GOAL',
        CartImpResult.OLD_HEADER_TIMESTAMP:'OLD_HEADER_TIMESTAMP',
        CartImpResult.PATH_TOLERANCE_VIOLATED:'PATH_TOLERANCE_VIOLATED',
        CartImpResult.GOAL_TOLERANCE_VIOLATED:'GOAL_TOLERANCE_VIOLATED',
        CartImpResult.UNKNOWN_ERROR:'UNKNOWN_ERROR', }

    _joint_trajectory_result_names = {
        FollowJointTrajectoryResult.SUCCESSFUL:"SUCCESSFUL",
        FollowJointTrajectoryResult.INVALID_GOAL:"INVALID_GOAL",
        FollowJointTrajectoryResult.INVALID_JOINTS:"INVALID_JOINTS",
        FollowJointTrajectoryResult.OLD_HEADER_TIMESTAMP:"OLD_HEADER_TIMESTAMP",
        FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:"PATH_TOLERANCE_VIOLATED",
        FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:"GOAL_TOLERANCE_VIOLATED",}

    _joint_trapezoid_trajectory_result_names = {
        TrapezoidTrajectoryResult.ACTIVE:"ACTIVE",
        TrapezoidTrajectoryResult.INACTIVE:"INACTIVE",
        TrapezoidTrajectoryResult.SUCCESSFUL:"SUCCESSFUL",
        TrapezoidTrajectoryResult.INVALID_GOAL:"INVALID_GOAL",
        TrapezoidTrajectoryResult.INVALID_JOINTS:"INVALID_JOINTS",
        TrapezoidTrajectoryResult.OLD_HEADER_TIMESTAMP:"OLD_HEADER_TIMESTAMP",
        TrapezoidTrajectoryResult.PATH_TOLERANCE_VIOLATED:"PATH_TOLERANCE_VIOLATED",
        TrapezoidTrajectoryResult.GOAL_TOLERANCE_VIOLATED:"GOAL_TOLERANCE_VIOLATED",
        TrapezoidTrajectoryResult.INVALID_LIMIT_ARRAY:"INVALID_LIMIT_ARRAY",
        TrapezoidTrajectoryResult.TRAJECTORY_NOT_FEASIBLE:"TRAJECTORY_NOT_FEASIBLE",
        TrapezoidTrajectoryResult.CANT_CALCULATE_COEFFS:"CANT_CALCULATE_COEFFS",
        TrapezoidTrajectoryResult.MAX_VEL_UNREACHEABLE:"MAX_VEL_UNREACHEABLE",
        TrapezoidTrajectoryResult.BREACHED_POS_LIMIT:"BREACHED_POS_LIMIT",
        TrapezoidTrajectoryResult.ACC_TOO_SMALL_FOR_DURATION:"ACC_TOO_SMALL_FOR_DURATION",
        TrapezoidTrajectoryResult.TRAJECTORY_JUMP_CVD:"DURATION_TOO_LONG",
        TrapezoidTrajectoryResult.TRAJECTORY_JUMP_ACV:"DURATION_TOO_SHORT",
        TrapezoidTrajectoryResult.IMPOSSIBLE_VELOCITY:"IMPOSSIBLE_VELOCITY",}

    _moveHand_action_error_codes_names = {
        0:"SUCCESSFUL",
        -1:"INVALID_DOF_NAME",
        -2:"INVALID_GOAL",
        -3:"CURRENT_LIMIT",
        -4:"PRESSURE_LIMIT",
        -5:"RESET_IS_ACTIVE"}

    _core_ve_name = "/velma_core_ve_body"
    _core_cs_name = "/velma_core_cs"
    _task_cs_name = "/velma_task_cs_ros_interface"

    _joint_groups = {"impedance_joints":['torso_0_joint', 'right_arm_0_joint', 'right_arm_1_joint',
        'right_arm_2_joint', 'right_arm_3_joint', 'right_arm_4_joint', 'right_arm_5_joint',
        'right_arm_6_joint', 'left_arm_0_joint', 'left_arm_1_joint', 'left_arm_2_joint',
        'left_arm_3_joint', 'left_arm_4_joint', 'left_arm_5_joint', 'left_arm_6_joint'],
        "right_arm":['right_arm_0_joint', 'right_arm_1_joint',
        'right_arm_2_joint', 'right_arm_3_joint', 'right_arm_4_joint', 'right_arm_5_joint',
        'right_arm_6_joint'],
        "left_arm":['left_arm_0_joint', 'left_arm_1_joint', 'left_arm_2_joint',
        'left_arm_3_joint', 'left_arm_4_joint', 'left_arm_5_joint', 'left_arm_6_joint']}

    def getLastJointState(self):
        """!
        Get the newest joint state.

        @return (rospy.Time, dictionary): A Pair of timestamp of the newest joint state and dictionary
        (with key=joint_name, value=joint_position), or None if there is no joint state data yet.

        """
        with self._joint_states_lock:
            if self._js_pos_history_idx >= 0:
                return copy.copy(self._js_pos_history[self._js_pos_history_idx])
        return None

    def getJointStateAtTime(self, time):
        """!
        Get interpolated joint state at a given time.

        @param time rospy.Time: Time of joint state to be computed.

        @return dictionary: A dictionary (with key=joint_name, value=joint_position) of interpolated joint positions
        or None, if time is not within current joint state history.

        """
        with self._joint_states_lock:
            if self._js_pos_history_idx < 0:
                return None
                
            hist_len = len(self._js_pos_history)
            for step in range(hist_len-1):
                h1_idx = (self._js_pos_history_idx - step - 1) % hist_len
                h2_idx = (self._js_pos_history_idx - step) % hist_len
                if self._js_pos_history[h1_idx] == None or self._js_pos_history[h2_idx] == None:
                    return None

                time1 = self._js_pos_history[h1_idx][0]
                time2 = self._js_pos_history[h2_idx][0]
                if time > time1 and time <= time2:
                    factor = (time - time1).to_sec() / (time2 - time1).to_sec()
                    js_pos = {}
                    for joint_name in self._js_pos_history[h1_idx][1]:
                        js_pos[joint_name] = self._js_pos_history[h1_idx][1][joint_name] * (1.0 - factor) + self._js_pos_history[h2_idx][1][joint_name] * factor
                    return js_pos
            return None

    # Private method used as callback for joint_states ROS topic.
    def _jointStatesCallback(self, data):
        with self._joint_states_lock:
            joint_idx = 0
            js_pos = {}
            for joint_name in data.name:
                js_pos[joint_name] = data.position[joint_idx]
                joint_idx += 1

            self._js_pos_history_idx = (self._js_pos_history_idx + 1) % len(self._js_pos_history)
            self._js_pos_history[self._js_pos_history_idx] = (data.header.stamp, copy.copy(js_pos))

    def getHandCurrentConfiguration(self, prefix):
        """!
        Get current configuration of a specified hand.

        @param prefix string: Hand name, can be one of two values ('left' or 'right').

        @return An 8-tuple of positions of all joints of the specified hand,
            or None if there is no valid joint state data. The sequence
            of joint positions is: FingerOneKnuckleOneJoint, FingerOneKnuckleTwoJoint,
            FingerOneKnuckleThreeJoint, FingerTwoKnuckleOneJoint, FingerTwoKnuckleTwoJoint,
            FingerTwoKnuckleThreeJoint, FingerThreeKnuckleTwoJoint,
            FingerThreeKnuckleThreeJoint.

        @exception NameError: If prefix is not 'left' or 'right'.
        """
        if prefix != 'left' and prefix != 'right':
            raise NameError('wrong prefix: ' + str(prefix))
        js = self.getLastJointState()
        if js == None:
            return None
        q = ( js[1][prefix+'_HandFingerOneKnuckleOneJoint'],
            js[1][prefix+'_HandFingerOneKnuckleTwoJoint'],
            js[1][prefix+'_HandFingerOneKnuckleThreeJoint'],
            js[1][prefix+'_HandFingerTwoKnuckleOneJoint'],
            js[1][prefix+'_HandFingerTwoKnuckleTwoJoint'],
            js[1][prefix+'_HandFingerTwoKnuckleThreeJoint'],
            js[1][prefix+'_HandFingerThreeKnuckleTwoJoint'],
            js[1][prefix+'_HandFingerThreeKnuckleThreeJoint'] )
        return q

    def getHandLeftCurrentConfiguration(self):
        """!
        Get current configuration of left hand.

        @return tuple: An 8-tuple of positions of all joints of left hand,
            or None if there is no valid joint state data.

        @see getHandCurrentConfiguration
        """
        return self.getHandCurrentConfiguration("left")

    def getHandRightCurrentConfiguration(self):
        """!
        Get current configuration of right hand.

        @return tuple: An 8-tuple of positions of all joints of right hand,
            or None if there is no valid joint state data.

        @see getHandCurrentConfiguration

        """
        return self.getHandCurrentConfiguration("right")

    def getHeadCurrentConfiguration(self):
        """!
        Get current configuration of neck.

        @return tuple: A 2-tuple of positions of neck joints,
            or None if there is no valid joint state data.
        """
        js = self.getLastJointState()
        if js == None:
            return None
        q = ( js[1]["head_pan_joint"],
            js[1]["head_tilt_joint"] )
        return q

    # Private method used in waitForInit.
    def _allActionsConnected(self):
        allConnected = True
        for side in self._action_move_hand_client_connected:
            if not self._action_move_hand_client_connected[side]:
                self._action_move_hand_client_connected[side] = self._action_move_hand_client[side].wait_for_server(rospy.Duration.from_sec(0.001))
            if not self._action_cart_traj_client_connected[side]:
                self._action_cart_traj_client_connected[side] = self._action_cart_traj_client[side].wait_for_server(rospy.Duration.from_sec(0.001))
            allConnected = allConnected and self._action_move_hand_client_connected[side] and\
                self._action_cart_traj_client_connected[side]
        if not self._action_joint_traj_client_connected:
            self._action_joint_traj_client_connected = self._action_joint_traj_client.wait_for_server(rospy.Duration.from_sec(0.001))
        allConnected = allConnected and self._action_joint_traj_client_connected

        if not self._action_head_joint_traj_client_connected:
            self._action_head_joint_traj_client_connected = self._action_head_joint_traj_client.wait_for_server(rospy.Duration.from_sec(0.001))
        allConnected = allConnected and self._action_head_joint_traj_client_connected

        if not self._action_joint_trapez_traj_client_connected:
            self._action_joint_trapez_traj_client_connected = self._action_joint_trapez_traj_client.wait_for_server(rospy.Duration.from_sec(0.001))
        allConnected = allConnected and self._action_joint_trapez_traj_client_connected

        if not self._action_head_joint_trapez_traj_client_connected:
            self._action_head_joint_trapez_traj_client_connected = self._action_head_joint_traj_client.wait_for_server(rospy.Duration.from_sec(0.001))
        allConnected = allConnected and self._action_head_joint_trapez_traj_client_connected

        if not self._action_safe_col_client_connected:
            self._action_safe_col_client_connected = self._action_safe_col_client.wait_for_server(rospy.Duration.from_sec(0.001))
        allConnected = allConnected and self._action_safe_col_client_connected

        for motor in self._action_motor_client_connected:
            if not self._action_motor_client_connected[motor]:
                self._action_motor_client_connected[motor] = self._action_motor_client[motor].wait_for_server(rospy.Duration.from_sec(0.001))
            allConnected = allConnected and self._action_motor_client_connected[motor]

        return allConnected

    def waitForInit(self, timeout_s = None):
        """!
        Wait for the interface until it is initialized.

        @param timeout_s float: Timeout in seconds.

        @return True if the interface was succesfully initialized within timeout, False otherwise.
        """
        time_start = time.time()
        can_break = False
        while not rospy.is_shutdown():
            with self._joint_states_lock:
                if self._js_pos_history_idx >= 0:
                    can_break = True
            diag_info = False
            try:
                diag = self.getCoreCsDiag()
                if len(diag.history) > 0:
                    diag_info = True
            except:
                pass
            if can_break and self._allActionsConnected() and diag_info:
                break
            rospy.sleep(0.1)
            time_now = time.time()
            if timeout_s and (time_now-time_start) > timeout_s:
                print "ERROR: waitForInit: ", can_break, self._action_move_hand_client_connected["right"],\
                    self._action_move_hand_client_connected["left"], self._action_cart_traj_client_connected["right"],\
                    self._action_cart_traj_client_connected["left"], self._action_joint_traj_client_connected,\
                    self._action_head_joint_traj_client_connected, self._action_motor_client_connected,\
                    self._action_head_joint_trapez_traj_client_connected, self._action_joint_trapez_traj_client_connected,\
                    self._action_safe_col_client_connected
                return False
        return True

    def waitForJointState(self, abs_time):
        """!
        Wait for reception of joint state ROS msg with time stamp greater or equal abs_time

        @param abs_time rospy.Time: absolute time.

        @return Returns True.
        """
        while not rospy.is_shutdown():
            js = self.getLastJointState()
            if (js[0] - abs_time).to_sec() > 0:
                break
            rospy.sleep(0.1)
        return True

    def getBodyJointLimits(self):
        """!
        Gets limits of joints of both arms and torso.
        Joints of neck and grippers are not included.
        The joints are used in impedance control (both joint and cartesian).

        @return dictionary: Returns a dictionary {name:(lower_limit, upper_limit)} that maps joint name to
        a 2-tupe with lower and upper joint limit.
        """
        return self._body_joint_limits

    def getHeadJointLimits(self):
        """!
        Gets limits of joints of neck.

        @return dictionary: Returns a dictionary {name:(lower_limit, upper_limit)} that maps joint name to
        a 2-tupe with lower and upper joint limit.
        """
        return self._head_joint_limits

    def __init__(self):
        """!
        Initialization of the interface.
        """

        self._listener = tf.TransformListener();

        # read the joint information from the ROS parameter server
        self._body_joint_names = rospy.get_param(self._task_cs_name+"/JntImpAction/joint_names")
        if self._body_joint_names == None or len(self._body_joint_names) != 15:
            raise RuntimeError("Could not read ROS param '" + self._task_cs_name+"/JntImpAction/joint_names'")
        body_joint_lower_limits = rospy.get_param(self._task_cs_name+"/JntImpAction/lower_limits")
        body_joint_upper_limits = rospy.get_param(self._task_cs_name+"/JntImpAction/upper_limits")
        self._body_joint_limits = {}
        for i in range(len(self._body_joint_names)):
            self._body_joint_limits[self._body_joint_names[i]] = (body_joint_lower_limits[i], body_joint_upper_limits[i])

        self._head_joint_names = rospy.get_param(self._task_cs_name+"/HeadAction/joint_names")
        head_joint_lower_limits = rospy.get_param(self._task_cs_name+"/HeadAction/lower_limits")
        head_joint_upper_limits = rospy.get_param(self._task_cs_name+"/HeadAction/upper_limits")
        self._head_joint_limits = {}
        for i in range(len(self._head_joint_names)):
            self._head_joint_limits[self._head_joint_names[i]] = (head_joint_lower_limits[i], head_joint_upper_limits[i])

        self._all_joint_names = rospy.get_param(self._task_cs_name+"/JntPub/joint_names")

        self._all_links = []

        robot_description_xml = rospy.get_param("/robot_description")
        #print robot_description_xml

        dom = minidom.parseString(robot_description_xml)
        robot = dom.getElementsByTagName("robot")
        if len(robot) != 1:
            raise Exception("Could not parse robot_description xml: wrong number of 'robot' elements.")
        links = robot[0].getElementsByTagName("link")
        for l in links:
            name = l.getAttribute("name")
            if name == None:
                raise Exception("Could not parse robot_description xml: link element has no name.")

            obj_link = self.Link()
            obj_link.name = name

            visual = l.getElementsByTagName("visual")
            for v in visual:
                origin = v.getElementsByTagName("origin")
                if len(origin) != 1:
                    raise Exception("Could not parse robot_description xml: wrong number of origin elements in link " + name)
                rpy = origin[0].getAttribute("rpy").split()
                xyz = origin[0].getAttribute("xyz").split()
                frame = PyKDL.Frame(PyKDL.Rotation.RPY(float(rpy[0]), float(rpy[1]), float(rpy[2])), PyKDL.Vector(float(xyz[0]), float(xyz[1]), float(xyz[2])))
                geometry = v.getElementsByTagName("geometry")
                if len(geometry) != 1:
                    raise Exception("Could not parse robot_description xml: wrong number of geometry elements in link " + name)
                mesh = geometry[0].getElementsByTagName("mesh")
                if len(mesh) == 1:
                    obj_visual = self.VisualMesh()
                    obj_visual.filename = mesh[0].getAttribute("filename")
                    #print "link:", name, " mesh:", obj_visual.filename
                    obj_visual.origin = frame
                    obj_link.visuals.append( obj_visual )
                else:
                    pass
                    #print "link:", name, " other visual"

            self._all_links.append(obj_link)

        self._js_pos_history = []
        for i in range(200):
            self._js_pos_history.append( None )
        self._js_pos_history_idx = -1

        self._joint_states_lock = threading.Lock()

        self._action_move_hand_client_connected = {'right':False, 'left':False}
        self._action_cart_traj_client_connected = {'right':False, 'left':False}
        self._action_joint_traj_client_connected = False
        self._action_head_joint_traj_client_connected = False
        self._action_joint_trapez_traj_client_connected = False
        self._action_head_joint_trapez_traj_client_connected = False
        self._action_safe_col_client_connected = False

        self._action_motor_client_connected = {'hp':False, 'ht':False, 't':False}

        # cartesian wrist trajectory for right arm
        self._action_cart_traj_client = {
            'right':actionlib.SimpleActionClient("/velma_task_cs_ros_interface/right_arm/cartesian_trajectory", CartImpAction),
            'left':actionlib.SimpleActionClient("/velma_task_cs_ros_interface/left_arm/cartesian_trajectory", CartImpAction)
            }

        # joint trajectory for arms and torso
        self._action_joint_traj_client = actionlib.SimpleActionClient("/velma_task_cs_ros_interface/spline_trajectory_action_joint", FollowJointTrajectoryAction)

        # joint trajectory for head
        self._action_head_joint_traj_client = actionlib.SimpleActionClient("/velma_task_cs_ros_interface/head_spline_trajectory_action_joint", FollowJointTrajectoryAction)

        # joint trapezoid trajectory for arms and torso
        self._action_joint_trapez_traj_client = actionlib.SimpleActionClient("/velma_task_cs_ros_interface/trapezoid_trajectory_action_joint", TrapezoidTrajectoryAction)

        # joint trapezoid trajectory for head
        self._action_head_joint_trapez_traj_client = actionlib.SimpleActionClient("/velma_task_cs_ros_interface/head_trapezoid_trajectory_action_joint", TrapezoidTrajectoryAction)

        self._action_safe_col_client = actionlib.SimpleActionClient("/velma_task_cs_ros_interface/safe_col_action", BehaviorSwitchAction)

        # motor actions for head
        self._action_motor_client = {
            'hp':actionlib.SimpleActionClient("/velma_task_cs_ros_interface/motors/hp", MotorAction),
            'ht':actionlib.SimpleActionClient("/velma_task_cs_ros_interface/motors/ht", MotorAction),
            't':actionlib.SimpleActionClient("/velma_task_cs_ros_interface/motors/t", MotorAction)
            }

        # cartesian tool trajectory for arms in the wrist frames
#        self._action_tool_client = {
#            'right':actionlib.SimpleActionClient("/right_arm/tool_trajectory", CartesianTrajectoryAction),
#            'left':actionlib.SimpleActionClient("/left_arm/tool_trajectory", CartesianTrajectoryAction) }
#        self._action_tool_client["right"].wait_for_server()
#        self._action_tool_client["left"].wait_for_server()

        # cartesian impedance trajectory for right arm
#        self._action_impedance_client = {
#            'right':actionlib.SimpleActionClient("/right_arm/cartesian_impedance", CartesianImpedanceAction),
#            'left':actionlib.SimpleActionClient("/left_arm/cartesian_impedance", CartesianImpedanceAction) }
#        self._action_impedance_client["right"].wait_for_server()
#        self._action_impedance_client["left"].wait_for_server()

        self._action_move_hand_client = {
            'right':actionlib.SimpleActionClient("/velma_task_cs_ros_interface/right_hand/move_hand", BHMoveAction),
            'left':actionlib.SimpleActionClient("/velma_task_cs_ros_interface/left_hand/move_hand", BHMoveAction) }
#        self._action_move_hand_client["right"].wait_for_server()    # this check is done in waitForInit
#        self._action_move_hand_client["left"].wait_for_server()     # this check is done in waitForInit

#        self.pub_reset_left = rospy.Publisher("/left_hand/reset_fingers", std_msgs.msg.Empty, queue_size=100)
#        self.pub_reset_right = rospy.Publisher("/right_hand/reset_fingers", std_msgs.msg.Empty, queue_size=100)

        #self.br = tf.TransformBroadcaster()

        rospy.sleep(1.0)
        
        #self.wrench_tab = []
        #self.wrench_tab_index = 0
        #self.wrench_tab_len = 4000
        #for i in range(0,self.wrench_tab_len):
        #    self.wrench_tab.append( Wrench(Vector3(), Vector3()) )

        self._listener_joint_states = rospy.Subscriber('/joint_states', JointState, self._jointStatesCallback)

        subscribed_topics_list = [
            ('/right_arm/transformed_wrench', geometry_msgs.msg.Wrench),
            ('/left_arm/transformed_wrench', geometry_msgs.msg.Wrench),
            ('/right_arm/wrench', geometry_msgs.msg.Wrench),
            ('/left_arm/wrench', geometry_msgs.msg.Wrench),
            ('/right_arm/ft_sensor/wrench', geometry_msgs.msg.Wrench),
            ('/left_arm/ft_sensor/wrench', geometry_msgs.msg.Wrench),
            (self._core_ve_name + "/diag", diagnostic_msgs.msg.DiagnosticArray),
            (self._core_cs_name + "/diag", diagnostic_msgs.msg.DiagnosticArray),]

        self._subscribed_topics = {}

        for topic in subscribed_topics_list:
            self._subscribed_topics[topic[0]] = [threading.Lock(), None]
            sub = rospy.Subscriber(topic[0], topic[1], partial( self._topicCallback, topic = topic[0] ))
            self._subscribed_topics[topic[0]].append(sub)

    def _getSubsystemDiag(self, subsystem_name):
        data = self._getTopicData(subsystem_name + "/diag")
        if data == None:
            return None
        for v in data.status[1].values:
            if v.key == "master_component":
                mcd = subsystem_common.parseMasterComponentDiag(v.value)
                return mcd
        return None

    def getCoreVeDiag(self):
        """!
        Get diagnostic information for core VE.

        @return Returns object of type subsystem_common.SubsystemDiag, with
            diagnostic information about subsystem.
        """
        return self._getSubsystemDiag(self._core_ve_name)

    class CoreCsDiag(subsystem_common.SubsystemDiag):
        """!
        This class contains subsystem-specific diagnostic information for velma_core_cs.
        """
        def __init__(self, parent):
            """!
            Initialization of diagnostics data using subsystem-independent diagnostics object.

            @param parent               subsystem_common.SubsystemDiag: subsystem-independent diagnostics object.
            @exception AssertionError   Raised when current state name cannot be obtained or state history is not present.
            """
            assert (len(parent.history) > 0)

            self.history = parent.history
            self.current_predicates = parent.current_predicates
            self.current_period = parent.current_period

            self.current_state = parent.history[0].state_name
            assert (self.current_state == "idle" or self.current_state == "safe" or
                self.current_state == "cart_imp" or self.current_state == "jnt_imp" or
                self.current_state == "safe_col" or self.current_state == "jnt_imp_trapez")

        def inStateIdle(self):
            """!
            Information about current state.
            @return True if the subsystem is in 'idle' state, False otherwise.
            """
            return self.current_state == "idle"

        def inStateSafe(self):
            """!
            Information about current state.
            @return True if the subsystem is in 'safe' state, False otherwise.
            """
            return self.current_state == "safe"

        def inStateCartImp(self):
            """!
            Information about current state.
            @return True if the subsystem is in 'cart_imp' state, False otherwise.
            """
            return self.current_state == "cart_imp"

        def inStateJntImp(self):
            """!
            Information about current state.
            @return True if the subsystem is in 'jnt_imp' state, False otherwise.
            """
            return self.current_state == "jnt_imp"

        def inStateJntImpTrapez(self):
            """!
            Information about current state.
            @return True if the subsystem is in 'jnt_imp_trapez' state, False otherwise.
            """
            return self.current_state == "jnt_imp_trapez"

        def inStateSafeCol(self):
            """!
            Information about current state.
            @return True if the subsystem is in 'safe_col' state, False otherwise.
            """
            return self.current_state == "safe_col"

        def isSafeReasonSelfCol(self):
            """!
            Information about reason for entering 'safe' state.
            @return True if the subsystem entered 'safe' state due to possibility of self-collision,
                False otherwise.
            @exception AssertionError   Raised when current state is not 'safe'.
            @exception KeyError         Raised when 'inSelfCollision' predicate cannot be obtained.
            """
            assert (self.current_state == "safe" and self.history[0].state_name == "safe")
            for pv in self.history[0].predicates:
                if pv.name == "inSelfCollision":
                    return pv.value
            raise KeyError("Could not obtain 'inSelfCollision' predicate value.")

        def isSafeReasonIdle(self):
            """!
            Information about reason for entering 'safe' state.
            @return True if the subsystem entered 'safe' state just after 'idle' state.
            @exception AssertionError   Raised when current state is not 'safe' or history contain only one entry.
            """
            assert (self.current_state == "safe" and self.history[0].state_name == "safe" and len(self.history) >= 2)

            return (self.history[1].state_name == "idle")

        def motorsReady(self):
            """!
            Information about state of head and torso motors.
            @return True if all motors are homed and ready to use, False otherwise.
            @exception KeyError   Raised when 'motorsReady' predicate cannot be obtained.
            """
            for pv in self.current_predicates:
                if pv.name == "motorsReady":
                    return pv.value
            raise KeyError("Could not obtain current 'motorsReady' predicate value.")

    def getCoreCsDiag(self):
        """!
        Get diagnostic information for core CS.

        @return Returns object of type VelmaInterface.CoreCsDiag, with
            diagnostic information about control subsystem.
        """
        return self.CoreCsDiag(self._getSubsystemDiag(self._core_cs_name))

    # Private method
    def _topicCallback(self, data, topic):
        with self._subscribed_topics[topic][0]:
            self._subscribed_topics[topic][1] = data

    # Private method
    def _getTopicData(self, topic):
        with self._subscribed_topics[topic][0]:
            if self._subscribed_topics[topic][1] != None:
                return copy.copy(self._subscribed_topics[topic][1])
        return None

    def getRawFTr(self):
        """!
        Gets right F/T sensor raw reading.

        @return PyKDL.Wrench: Returns KDL wrench.
        """
        return self._wrenchROStoKDL( self._getTopicData('/right_arm/ft_sensor/wrench') )

    def getRawFTl(self):
        """!
        Gets left F/T sensor raw reading.

        @return PyKDL.Wrench: Returns KDL wrench.
        """
        return self._wrenchROStoKDL( self._getTopicData('/left_arm/ft_sensor/wrench') )

    def getTransformedFTr(self):
        """!
        Gets right F/T sensor transformed reading.

        @return PyKDL.Wrench: Returns KDL wrench.
        """
        return self._wrenchROStoKDL( self._getTopicData('/right_arm/transformed_wrench') )

    def getTransformedFTl(self):
        """!
        Gets left F/T sensor transformed reading.

        @return PyKDL.Wrench: Returns KDL wrench.
        """
        return self._wrenchROStoKDL( self._getTopicData('/left_arm/transformed_wrench') )

    def getWristWrenchr(self):
        """!
        Gets estimated wrench for the right end effector.

        @return PyKDL.Wrench: Returns KDL wrench.
        """
        return self._wrenchROStoKDL( self._getTopicData('/right_arm/wrench') )

    def getWristWrenchl(self):
        """!
        Gets estimated wrench for the left end effector.

        @return PyKDL.Wrench: Returns KDL wrench.
        """
        return self._wrenchROStoKDL( self._getTopicData('/left_arm/wrench') )

    # Private method
    def _action_right_cart_traj_feedback_cb(self, feedback):
        self._action_right_cart_traj_feedback = copy.deepcopy(feedback)

    # Private method
    def _wrenchKDLtoROS(self, wrKDL):
        return geometry_msgs.msg.Wrench(Vector3( wrKDL.force.x(), wrKDL.force.y(), wrKDL.force.z() ), Vector3( wrKDL.torque.x(), wrKDL.torque.y(), wrKDL.torque.z() ))

    # Private method
    def _wrenchROStoKDL(self, wrROS):
        return PyKDL.Wrench( PyKDL.Vector(wrROS.force.x, wrROS.force.y, wrROS.force.z), PyKDL.Vector(wrROS.torque.x, wrROS.torque.y, wrROS.torque.z) )

    def switchToSafeColBehavior(self):
        """!
        Switches the robot to SafeCol behavior.
        """
        goal = BehaviorSwitchGoal()
        goal.command = 1
        self._action_safe_col_client.send_goal(goal)

    def enableMotor(self, motor):
        """!
        Enable motor.

        @param motor string: Name of motor to enable: 'hp', 'ht' or 't'

        @exception NameError: Parameter motor has invalid value.
        """
        if motor != 'hp' and motor != 'ht' and motor != 't':
            raise NameError(motor)
        goal = MotorGoal()
        goal.action = MotorGoal.ACTION_ENABLE
        self._action_motor_client[motor].send_goal(goal)

    def startHomingMotor(self, motor):
        """!
        Start homing procedure for specified motor.

        @param motor string: Name of motor to enable: 'hp' or 'ht'

        @exception NameError: Parameter motor has invalid value.
        """
        if motor != 'hp' and motor != 'ht':
            raise NameError(motor)
        goal = MotorGoal()
        goal.action = MotorGoal.ACTION_START_HOMING
        self._action_motor_client[motor].send_goal(goal)

    def waitForMotor(self, motor, timeout_s=0):
        """!
        Wait for an action for a motor to complete.

        @param motor string: Name of motor to enable: 'hp', 'ht' or 't'
        @param timeout_s float: Timeout in seconds.

        @return Returns error code or None if timed out.

        @exception NameError: Parameter motor has invalid value.
        """
        if motor != 'hp' and motor != 'ht' and motor != 't':
            raise NameError(motor)

        if not self._action_motor_client[motor].wait_for_result(timeout=rospy.Duration(timeout_s)):
            return None
        result = self._action_motor_client[motor].get_result()

        error_code = result.error_code

        if error_code == MotorResult.ERROR_ALREADY_ENABLED:
            print "waitForMotor('" + motor + "'): ERROR_ALREADY_ENABLED (no error)"
            error_code = 0
        elif error_code == MotorResult.ERROR_HOMING_DONE:
            print "waitForMotor('" + motor + "'): ERROR_HOMING_DONE (no error)"
            error_code = 0
        if error_code != 0:
            print "waitForMotor('" + motor + "'): action failed with error_code=" + str(result.error_code)
        return error_code

    def enableMotors(self, timeout=0):
        """!
        Enable all motors and wait.

        @param timeout_s float: Timeout in seconds.

        @see enableMotor
        """
        self.enableMotor("hp")
        r_hp = self.waitForMotor("hp", timeout_s=timeout)
        self.enableMotor("ht")
        r_ht = self.waitForMotor("ht", timeout_s=timeout)
        self.enableMotor("t")
        r_t = self.waitForMotor("t", timeout_s=timeout)
        if r_hp != 0:
            return r_hp
        if r_ht != 0:
            return r_ht
        return r_t

    def startHomingHP(self):
        """!
        Start homing procedure for head pan motor.

        @see startHomingMotor
        """
        self.startHomingMotor("hp")

    def startHomingHT(self):
        """!
        Start homing procedure for head tilt motor.

        @see startHomingMotor
        """
        self.startHomingMotor("hp")
        self.startHomingMotor("ht")

    def waitForHP(self, timeout_s=0):
        """!
        Wait for head pan motor.

        @param timeout_s float: Timeout in seconds.

        @see waitForMotor
        """
        return self.waitForMotor("hp", timeout_s=timeout_s)

    def waitForHT(self, timeout_s=0):
        """!
        Wait for head tilt motor.

        @param timeout_s float: Timeout in seconds.

        @see waitForMotor
        """
        return self.waitForMotor("ht", timeout_s=timeout_s)

    def waitForT(self, timeout_s=0):
        """!
        Wait for torso motor.

        @param timeout_s float: Timeout in seconds.

        @see waitForMotor
        """
        return self.waitForMotor("t", timeout_s=timeout_s)

    def moveCartImp(self, prefix, pose_list_T_B_Td, pose_times, tool_list_T_W_T, tool_times, imp_list, imp_times, max_wrench, start_time=0.01, stamp=None, damping=PyKDL.Wrench(PyKDL.Vector(0.7, 0.7, 0.7),PyKDL.Vector(0.7, 0.7, 0.7)), path_tol=None):
        """!
        Execute motion in cartesian impedance mode.

        @param prefix           string: name of end-effector: 'left' or 'right'.
        @param pose_list_T_B_Td list of PyKDL.Frame: Poses of end-effector tool wrt. base frame. This is a path for the end-effector.
        @param pose_times       list of float: Times of the poses in pose_list_T_B_Td wrt. start_time.
        @param tool_list_T_W_T  list of PyKDL.Frame: Poses of tool frame wrt. wrist frame. This is a path for the tool of end-effector.
        @param tool_times       list of float: Times of the tool poses in tool_list_T_W_T wrt. start_time.
        @param imp_list         list of PyKDL.Wrench: Tool impedance values expressed in tool frame. This is a path for the impedance of end-effector.
        @param imp_times        list of float: Times of the impedance values in imp_list wrt. start_time.
        @param max_wrench       PyKDL.Wrench: Maximum allowed wrench during the motion expressed in tool frame.
        @param start_time       float: Relative start time for the movement.
        @param stamp            rospy.Time: Absolute start time for the movement. If both start_time and stamp arguments are set, only stamp is taken into account.
        @param damping          PyKDL.Wrench: Damping for the tool expressed in tool frame.
        @param path_tol         PyKDL.Twist: Maximum allowed error of tool pose expressed in the tool frame. Error of tool pose is a difference between desired tool equilibrium pose and measured tool pose.

        @return Returns True.

        @exception AssertionError   Raised when prefix is neither 'left' nor 'right'.
        """

        assert (prefix=="left" or prefix=="right")
        assert ((pose_list_T_B_Td == None and pose_times == None) or len(pose_list_T_B_Td) == len(pose_times))
        assert ((tool_list_T_W_T == None and tool_times == None) or len(tool_list_T_W_T) == len(tool_times))
        assert ((imp_list == None and imp_times == None) or len(imp_list) == len(imp_times))

        action_trajectory_goal = CartImpGoal()
        if stamp == None:
            stamp = rospy.Time.now() + rospy.Duration(start_time)

        action_trajectory_goal.pose_trj.header.stamp = stamp
        action_trajectory_goal.tool_trj.header.stamp = stamp
        action_trajectory_goal.imp_trj.header.stamp = stamp

        if pose_list_T_B_Td != None:
            for i in range(len(pose_list_T_B_Td)):
                wrist_pose = pm.toMsg(pose_list_T_B_Td[i])
                action_trajectory_goal.pose_trj.points.append(CartesianTrajectoryPoint(
                rospy.Duration(pose_times[i]),
                wrist_pose,
                Twist()))

        if tool_list_T_W_T != None:
            for i in range(len(tool_list_T_W_T)):
                tool_pose = pm.toMsg(tool_list_T_W_T[i])
                action_trajectory_goal.tool_trj.points.append(CartesianTrajectoryPoint(
                rospy.Duration(tool_times[i]),
                tool_pose,
                Twist()))

        if imp_list != None:
            for i in range(len(imp_list)):
                action_trajectory_goal.imp_trj.points.append(CartesianImpedanceTrajectoryPoint(
                rospy.Duration(imp_times[i]),
                CartesianImpedance(self._wrenchKDLtoROS(imp_list[i]), self._wrenchKDLtoROS(damping))))

        if path_tol != None:
            action_trajectory_goal.path_tolerance.position = geometry_msgs.msg.Vector3( path_tol.vel.x(), path_tol.vel.y(), path_tol.vel.z() )
            action_trajectory_goal.path_tolerance.rotation = geometry_msgs.msg.Vector3( path_tol.rot.x(), path_tol.rot.y(), path_tol.rot.z() )
            action_trajectory_goal.goal_tolerance.position = geometry_msgs.msg.Vector3( path_tol.vel.x(), path_tol.vel.y(), path_tol.vel.z() )
            action_trajectory_goal.goal_tolerance.rotation = geometry_msgs.msg.Vector3( path_tol.rot.x(), path_tol.rot.y(), path_tol.rot.z() )

        action_trajectory_goal.wrench_constraint = self._wrenchKDLtoROS(max_wrench)
        self._action_cart_traj_client[prefix].send_goal(action_trajectory_goal)

        return True

    def moveCartImpRight(self, pose_list_T_B_Td, pose_times, tool_list_T_W_T, tool_times, imp_list, imp_times, max_wrench, start_time=0.01, stamp=None, damping=PyKDL.Wrench(PyKDL.Vector(0.7, 0.7, 0.7),PyKDL.Vector(0.7, 0.7, 0.7)), path_tol=None):
        """!
        Execute motion in cartesian impedance mode for the right end-effector.
        @see moveCartImp
        """
        return self.moveCartImp("right", pose_list_T_B_Td, pose_times, tool_list_T_W_T, tool_times, imp_list, imp_times, max_wrench, start_time=start_time, stamp=stamp, damping=damping, path_tol=path_tol)

    def moveCartImpLeft(self, pose_list_T_B_Td, pose_times, tool_list_T_W_T, tool_times, imp_list, imp_times, max_wrench, start_time=0.01, stamp=None, damping=PyKDL.Wrench(PyKDL.Vector(0.7, 0.7, 0.7),PyKDL.Vector(0.7, 0.7, 0.7)), path_tol=None):
        """!
        Execute motion in cartesian impedance mode for the left end-effector.
        @see moveCartImp
        """
        return self.moveCartImp("left", pose_list_T_B_Td, pose_times, tool_list_T_W_T, tool_times, imp_list, imp_times, max_wrench, start_time=start_time, stamp=stamp, damping=damping, path_tol=path_tol)

    def moveCartImpRightCurrentPos(self, start_time=0.2, stamp=None):
        """!
        Move right end-effector to current position. Switch core_cs to cart_imp mode.
        @return Returns True.
        """
        return self.moveCartImp("right", None, None, None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=start_time, stamp=stamp)

    def moveCartImpLeftCurrentPos(self, start_time=0.2, stamp=None):
        """!
        Move left end-effector to current position. Switch core_cs to cart_imp mode.
        @return Returns True.
        """
        return self.moveCartImp("left", None, None, None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=start_time, stamp=stamp)

    def waitForEffector(self, prefix, timeout_s=None):
        """!
        Wait for completion of end-effector motion in cartesian impedance mode.

        @param prefix       string: name of end-effector: 'left' or 'right'.
        @param timeout_s    float: timeout in seconds.

        @return Returns error code.

        @exception AssertionError   Raised when prefix is neither 'left' nor 'right'
        """
        assert (prefix=="left" or prefix=="right")

        if timeout_s == None:
            timeout_s = 0
        if not self._action_cart_traj_client[prefix].wait_for_result(timeout=rospy.Duration(timeout_s)):
            return None
        result = self._action_cart_traj_client[prefix].get_result()
        if result.error_code != 0:
            error_str = "UNKNOWN"
            if result.error_code in self._cartesian_trajectory_result_names:
                error_str = self._cartesian_trajectory_result_names[result.error_code]
            print "waitForEffector(" + prefix + "): action failed with error_code=" + str(result.error_code) + " (" + error_str + ")"

        self.waitForJointState( self._action_cart_traj_client[prefix].gh.comm_state_machine.latest_result.header.stamp )

        return result.error_code

    def waitForEffectorLeft(self, timeout_s=None):
        """!
        Wait for completion of left end-effector motion in cartesian impedance mode.
        @param timeout_s    float: Timeout in seconds.
        @return Returns error code.
        """
        return self.waitForEffector("left", timeout_s=timeout_s)

    def waitForEffectorRight(self, timeout_s=None):
        """!
        Wait for completion of right end-effector motion in cartesian impedance mode.
        @param timeout_s    float: Timeout in seconds.
        @return Returns error code.
        """
        return self.waitForEffector("right", timeout_s=timeout_s)

    def maxJointTrajLen(self):
        """!
        Get maximum number of nodes in single joint trajectory.
        @return Maximum number of nodes.
        """
        return 50

    def maxHeadTrajLen(self):
        """!
        Get maximum number of nodes in single head trajectory.
        @return Maximum number of nodes.
        """
        return 50

    def getJointGroup(self, group_name):
        """!
        Get names of all joints in group.
        @param group_name   string: name of group.
        @return Returns list of names of joints in group.
        """
        return self._joint_groups[group_name]

    def moveJointTraj(self, traj, start_time=0.2, stamp=None, position_tol=5.0/180.0 * math.pi, velocity_tol=5.0/180.0*math.pi):
        """!
        Execute joint space trajectory in joint impedance mode.
        @param traj         trajectory_msgs.msg.JointTrajectory: joint trajectory.
        @param start_time   float: relative start time.
        @param stamp        rospy.Time: absolute start time.
        @position_tol       float: position tolerance.
        @velocity_tol       float: velocity tolerance.
        @return Returns True.
        @exception AssertionError   Raised when trajectory has too many nodes.
        """
        assert (len(traj.points) <= self.maxJointTrajLen())

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self._body_joint_names

        js = self.getLastJointState()
        assert (js != None)

        for node_idx in range(len(traj.points)):
            q_dest_all = []
            vel_dest_all = []

            for joint_name in self._body_joint_names:
                if joint_name in traj.joint_names:
                    q_idx = traj.joint_names.index(joint_name)
                    q_dest_all.append(traj.points[node_idx].positions[q_idx])
                    if len(traj.points[node_idx].velocities) > 0:
                        vel_dest_all.append(traj.points[node_idx].velocities[q_idx])
                    else:
                        vel_dest_all.append(0)
                else:
                    q_dest_all.append(js[1][joint_name])
                    vel_dest_all.append(0)
            goal.trajectory.points.append(JointTrajectoryPoint(q_dest_all, vel_dest_all, [], [], traj.points[node_idx].time_from_start))

        for joint_name in goal.trajectory.joint_names:
            goal.path_tolerance.append(JointTolerance(joint_name, position_tol, velocity_tol, 0.0))
        if stamp != None:
            goal.trajectory.header.stamp = stamp
        else:
            goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(start_time)
        self._action_joint_traj_client.send_goal(goal)
        return True

    def moveJoint(self, q_dest_map, time, start_time=0.2, stamp=None, position_tol=5.0/180.0 * math.pi, velocity_tol=5.0/180.0*math.pi):
        """!
        Execute simple joint space motion in joint impedance mode.
        @param q_dest_map   dictionary: dictionary {name:position} of goal configuration
        @param start_time   float: relative start time.
        @param stamp        rospy.Time: absolute start time.
        @position_tol       float: position tolerance.
        @velocity_tol       float: velocity tolerance.
        @return Returns True.
        """
        traj = JointTrajectory()
        pt = JointTrajectoryPoint()
        for name in q_dest_map:
            traj.joint_names.append(name)
            pt.positions.append(q_dest_map[name])
            pt.velocities.append(0)
        pt.time_from_start = rospy.Duration(time)
        traj.points.append(pt)
        return self.moveJointTraj(traj, start_time=start_time, stamp=stamp, position_tol=position_tol, velocity_tol=velocity_tol)

    def moveJointImpToCurrentPos(self, start_time=0.2, stamp=None):
        """!
        Switch core_cs to jnt_imp mode.
        @return Returns True.
        """
        traj = JointTrajectory()
        return self.moveJointTraj(traj, start_time=start_time, stamp=stamp)

    def waitForJoint(self):
        """!
        Wait for joint space movement to complete.
        @return Returns error code.
        """
        self._action_joint_traj_client.wait_for_result()
        result = self._action_joint_traj_client.get_result()
        if result.error_code != 0:
            error_str = "UNKNOWN"
            if result.error_code in self._joint_trajectory_result_names:
                error_str = self._joint_trajectory_result_names[result.error_code]
            print "waitForJoint(): action failed with error_code=" + str(result.error_code) + " (" + error_str + ")"

        self.waitForJointState( self._action_joint_traj_client.gh.comm_state_machine.latest_result.header.stamp )

        return result.error_code

    def moveHeadTraj(self, traj, start_time=0.2, stamp=None, position_tol=5.0/180.0 * math.pi, velocity_tol=5.0/180.0*math.pi):
        """!
        Execute head trajectory in joint space.
        @param traj         trajectory_msgs.msg.JointTrajectory: joint trajectory.
        @param start_time   float: relative start time.
        @param stamp        rospy.Time: absolute start time.
        @position_tol       float: position tolerance.
        @velocity_tol       float: velocity tolerance.
        @return Returns True.
        @exception AssertionError   Raised when trajectory has too many nodes or not all joints are
            included in trajectory.
        """
        assert (len(traj.points) <= self.maxHeadTrajLen())

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["head_pan_joint", "head_tilt_joint"]

        for node_idx in range(len(traj.points)):
            q_dest_all = []
            vel_dest_all = []
            for joint_name in goal.trajectory.joint_names:
                assert (joint_name in traj.joint_names)
                q_idx = traj.joint_names.index(joint_name)
                q_dest_all.append(traj.points[node_idx].positions[q_idx])
                if len(traj.points[node_idx].velocities) > 0:
                    vel_dest_all.append(traj.points[node_idx].velocities[q_idx])
                else:
                    vel_dest_all.append(0)
            goal.trajectory.points.append(JointTrajectoryPoint(q_dest_all, vel_dest_all, [], [], traj.points[node_idx].time_from_start))

        for joint_name in goal.trajectory.joint_names:
            goal.path_tolerance.append(JointTolerance(joint_name, position_tol, velocity_tol, 0))
        if stamp != None:
            goal.trajectory.header.stamp = stamp
        else:
            goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(start_time)
        self._action_head_joint_traj_client.send_goal(goal)
        return True

    def moveHead(self, q_dest, time, start_time=0.2, stamp=None, position_tol=5.0/180.0 * math.pi, velocity_tol=5.0/180.0*math.pi):
        """!
        Execute simple head motion in joint space.
        @param q_dest       2-tuple:  goal configuration of head ('head_pan_joint', 'head_tilt_joint')
        @param start_time   float: relative start time.
        @param stamp        rospy.Time: absolute start time.
        @position_tol       float: position tolerance.
        @velocity_tol       float: velocity tolerance.
        @return Returns True.
        @exception AssertionError   When q_dest has wrong length.
        """
        assert (len(q_dest) == 2)
        traj = JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = q_dest
        pt.velocities = [0.0, 0.0]
        pt.time_from_start = rospy.Duration(time)
        traj.points.append(pt)
        return self.moveHeadTraj(traj, start_time=start_time, stamp=stamp, position_tol=position_tol, velocity_tol=velocity_tol)

    def waitForHead(self):
        """!
        Wait for head movement to complete.
        @return Returns error code.
        """
        self._action_head_joint_traj_client.wait_for_result()
        result = self._action_head_joint_traj_client.get_result()
        if result.error_code != 0:
            print "waitForHead(): action failed with error_code=" + str(result.error_code)

        self.waitForJointState( self._action_head_joint_traj_client.gh.comm_state_machine.latest_result.header.stamp )

        return result.error_code

    def moveJointTrajTrapez(self, traj, research_mode, duration_mode, save_data, max_velocities=None, max_accelerations=None, start_time=0.2, stamp=None, position_tol=5.0/180.0 * math.pi, velocity_tol=5.0/180.0*math.pi):
        """!
        Execute joint space trapezoid trajectory in joint impedance mode.
        @param traj         trajectory_msgs.msg.JointTrajectory: joint trajectory.
        @param start_time   float: relative start time.
        @param stamp        rospy.Time: absolute start time.
        @position_tol       float: position tolerance.
        @velocity_tol       float: velocity tolerance.
        @return Returns True.
        @exception AssertionError   Raised when trajectory has too many nodes.
        """
        assert (len(traj.points) <= self.maxJointTrajLen())
        
        goal = TrapezoidTrajectoryGoal()
        goal.trajectory.joint_names = self._body_joint_names

        js = self.getLastJointState()
        assert (js != None)
        for node_idx in range(len(traj.points)):
            q_dest_all = []
            vel_dest_all = []

            for joint_name in self._body_joint_names:
                if joint_name in traj.joint_names:
                    q_idx = traj.joint_names.index(joint_name)
                    q_dest_all.append(traj.points[node_idx].positions[q_idx])
                    if len(traj.points[node_idx].velocities) > 0:
                        vel_dest_all.append(traj.points[node_idx].velocities[q_idx])
                    else:
                        vel_dest_all.append(0)
                else:
                    q_dest_all.append(js[1][joint_name])
                    vel_dest_all.append(0)
            goal.trajectory.points.append(JointTrajectoryPoint(q_dest_all, vel_dest_all, [], [], rospy.Duration(3.0)))
        if max_velocities !=None:
            goal.max_velocities = max_velocities
        if max_accelerations !=None:
            goal.max_accelerations = max_accelerations

        goal.duration_mode = duration_mode
        goal.research_mode = research_mode
        goal.save_data = save_data
        for joint_name in goal.trajectory.joint_names:
            goal.path_tolerance.append(JointTolerance(joint_name, position_tol, velocity_tol, 0.0))

        if stamp != None:
            goal.trajectory.header.stamp = stamp
        else:
            goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(start_time)
        self._action_joint_trapez_traj_client.send_goal(goal)
        return True

    def moveJointTrapez(self, q_dest_map, research_mode, duration_mode, save_data, max_velocities=None, max_accelerations=None, start_time=0.2, stamp=None, position_tol=5.0/180.0 * math.pi, velocity_tol=5.0/180.0*math.pi):
        """!
        Execute simple joint space motion in joint impedance mode.
        @param q_dest_map   dictionary: dictionary {name:position} of goal configuration
        @param start_time   float: relative start time.
        @param stamp        rospy.Time: absolute start time.
        @position_tol       float: position tolerance.
        @velocity_tol       float: velocity tolerance.
        @return Returns True.
        """
        traj = JointTrajectory()
        pt = JointTrajectoryPoint()
        for name in q_dest_map:
            traj.joint_names.append(name)
            pt.positions.append(q_dest_map[name])
            pt.velocities.append(0)
        pt.time_from_start = rospy.Duration(3.0)
        traj.points.append(pt)
        return self.moveJointTrajTrapez(traj, research_mode, duration_mode, save_data, max_velocities, max_accelerations, start_time=start_time, stamp=stamp, position_tol=position_tol, velocity_tol=velocity_tol)

    def moveJointImpTrapezToCurrentPos(self, start_time=0.2, stamp=None):
        """!
        Switch core_cs to jnt_imp_trapez mode.
        @return Returns True.
        """
        traj = JointTrajectory()
        return self.moveJointTrajTrapez(traj,False, True, False, None, None, start_time=start_time, stamp=stamp)

    def waitForJointTrapez(self):
        """!
        Wait for joint space movement to complete.
        @return Returns error code.
        """
        self._action_joint_trapez_traj_client.wait_for_result()
        result = self._action_joint_trapez_traj_client.get_result()
        if result.error_code != 0:
            error_str = "UNKNOWN"
            if result.error_code in self._joint_trapezoid_trajectory_result_names:
                error_str = self._joint_trapezoid_trajectory_result_names[result.error_code]
            print "waitForJointTrapez(): action failed with error_code=" + str(result.error_code) + " (" + error_str + ")"
        self.waitForJointState( self._action_joint_trapez_traj_client.gh.comm_state_machine.latest_result.header.stamp )

        return result.error_code

    def moveHeadTrajTrapez(self, traj, research_mode, duration_mode, save_data, max_velocities=None, max_accelerations=None, start_time=0.2, stamp=None, position_tol=5.0/180.0 * math.pi, velocity_tol=5.0/180.0*math.pi):
        """!
        Execute head trajectory in joint space.
        @param traj         trajectory_msgs.msg.JointTrajectory: joint trajectory.
        @param start_time   float: relative start time.
        @param stamp        rospy.Time: absolute start time.
        @position_tol       float: position tolerance.
        @velocity_tol       float: velocity tolerance.
        @return Returns True.
        @exception AssertionError   Raised when trajectory has too many nodes or not all joints are
            included in trajectory.
        """
        assert (len(traj.points) <= self.maxHeadTrajLen())

        goal = TrapezoidTrajectoryGoal()
        goal.trajectory.joint_names = ["head_pan_joint", "head_tilt_joint"]

        for node_idx in range(len(traj.points)):
            q_dest_all = []
            vel_dest_all = []
            for joint_name in goal.trajectory.joint_names:
                assert (joint_name in traj.joint_names)
                q_idx = traj.joint_names.index(joint_name)
                q_dest_all.append(traj.points[node_idx].positions[q_idx])
                if len(traj.points[node_idx].velocities) > 0:
                    vel_dest_all.append(traj.points[node_idx].velocities[q_idx])
                else:
                    vel_dest_all.append(0)
            goal.trajectory.points.append(JointTrajectoryPoint(q_dest_all, vel_dest_all, [], [], rospy.Duration(3.0)))

        if duration_mode:
            goal.max_velocities = max_velocities
            goal.max_accelerations = max_accelerations
        else:
            goal.max_velocities = []
            goal.max_accelerations = []

        goal.duration_mode = duration_mode
        goal.research_mode = research_mode
        goal.save_data = save_data

        for joint_name in goal.goal.trajectory.joint_names:
            goal.path_tolerance.append(JointTolerance(joint_name, position_tol, velocity_tol, 0))
        if stamp != None:
            goal.trajectory.header.stamp = stamp
        else:
            goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(start_time)
        self._action_head_joint_trapez_traj_client.send_goal(goal)
        return True

    def moveHeadTrapez(self, q_dest, research_mode, duration_mode, save_data, max_velocities=None, max_accelerations=None, start_time=0.2, stamp=None, position_tol=5.0/180.0 * math.pi, velocity_tol=5.0/180.0*math.pi):
        """!
        Execute simple head motion in joint space.
        @param q_dest       2-tuple:  goal configuration of head ('head_pan_joint', 'head_tilt_joint')
        @param start_time   float: relative start time.
        @param stamp        rospy.Time: absolute start time.
        @position_tol       float: position tolerance.
        @velocity_tol       float: velocity tolerance.
        @return Returns True.
        @exception AssertionError   When q_dest has wrong length.
        """
        assert (len(q_dest) == 2)
        traj = JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = q_dest
        pt.velocities = [0.0, 0.0]
        pt.time_from_start = rospy.Duration(3.0)
        traj.points.append(pt)
        return self.moveHeadTrajTrapez(traj, research_mode, duration_mode, save_data, max_velocities, max_accelerations, start_time=start_time, stamp=stamp, position_tol=position_tol, velocity_tol=velocity_tol)

    def waitForHeadTrapez(self):
        """!
        Wait for head movement to complete.
        @return Returns error code.
        """
        self._action_head_joint_trapez_traj_client.wait_for_result()
        result = self._action_head_joint_trapez_traj_client.get_result().result
        if result.error_code != 0:
            print "waitForHead(): action failed with error_code=" + str(result.error_code)

        self.waitForJointState( self._action_head_joint_trapez_traj_client.gh.comm_state_machine.latest_result.header.stamp )

        return result.error_code

    def moveHand(self, prefix, q, v, t, maxPressure, hold=False):
        """!
        Executes hand motion with trapezoidal velocity profile.

        @param prefix       string: name of hand, either 'left' or 'right'
        @param q            4-tuple of float: desired configuration for hand DOFs:
            [FingerOneKnuckleTwo, FingerTwoKnuckleTwo, FingerThreeKnuckleTwo, Spread]
        @param v            4-tuple of float: maximum velocities
        @param t            4-tuple of float: maximum current in motors
        @param maxPressure  float: maximum pressure for tactile sensors
        @param hold         bool: True if spread joint should hold its position after completion of motion
        @exception AssertionError when prefix is neither 'left' nor 'right'
        """
        assert (prefix == 'left' or prefix == 'right')
        action_goal = BHMoveGoal()
        action_goal.name = [prefix+"_HandFingerOneKnuckleTwoJoint", prefix+"_HandFingerTwoKnuckleTwoJoint",
            prefix+"_HandFingerThreeKnuckleTwoJoint", prefix+"_HandFingerOneKnuckleOneJoint"]
        action_goal.q = q
        action_goal.v = v
        action_goal.t = t
        action_goal.maxPressure = maxPressure
        action_goal.reset = False
        if hold == True:
            action_goal.hold = 1
        else:
            action_goal.hold = 0
        self._action_move_hand_client[prefix].send_goal(action_goal)

    def moveHandLeft(self, q, v, t, maxPressure, hold=False):
        """!
        Executes motion with trapezoidal velocity profile for left hand.
        @see moveHand
        """
        self.moveHand("left", q, v, t, maxPressure, hold=hold)

    def moveHandRight(self, q, v, t, maxPressure, hold=False):
        """!
        Executes motion with trapezoidal velocity profile for right hand.
        @see moveHand
        """
        self.moveHand("right", q, v, t, maxPressure, hold=hold)

    def resetHand(self, prefix):
        """!
        Executes reset command for hand.
        @param prefix       string: name of hand, either 'left' or 'right'
        @exception AssertionError when prefix is neither 'left' nor 'right'
        """
        assert (prefix == 'left' or prefix == 'right')
        action_goal = BHMoveGoal()
        action_goal.reset = True
        self._action_move_hand_client[prefix].send_goal(action_goal)

    def resetHandLeft(self):
        """!
        Executes reset command for left hand.
        @see resetHand
        """
        self.resetHand("left")

    def resetHandRight(self):
        """!
        Executes reset command for right hand.
        @see resetHand
        """
        self.resetHand("right")

    def waitForHand(self, prefix):
        """!
        Wait for completion of hand movement.
        @param prefix       string: name of hand, either 'left' or 'right'
        @exception AssertionError when prefix is neither 'left' nor 'right'
        """
        assert (prefix == 'left' or prefix == 'right')
        self._action_move_hand_client[prefix].wait_for_result()
        result = self._action_move_hand_client[prefix].get_result()
        if result.error_code != 0:
            print "waitForHand(" + prefix + "): action failed with error_code=" + str(result.error_code) + " (" + self._moveHand_action_error_codes_names[result.error_code] + ")"
        return result.error_code

    def waitForHandLeft(self):
        """!
        Wait for completion of left hand movement.
        @see waitForHand
        """
        return self.waitForHand("left")

    def waitForHandRight(self):
        """!
        Wait for completion of right hand movement.
        @see waitForHand
        """
        return self.waitForHand("right")

    def _getKDLtf(self, base_frame, frame, time=None, timeout_s=1.0):
        """!
        Lookup tf transform and convert it to PyKDL.Frame.
        @param base_frame   string: name of base frame
        @param frame        string: name of target frame
        @param time         rospy.Time: time at which transform should be interpolated
        @param timeout_s    float: timeout in seconds
        @return PyKDL.Frame transformation from base frame to target frame.
        @exception tf2_ros.TransformException when the transform for the specified time
            is not avaible during the timeout_s.
        """
        if time == None:
            time = rospy.Time.now()
        self._listener.waitForTransform(base_frame, frame, time, rospy.Duration(timeout_s))
        pose = self._listener.lookupTransform(base_frame, frame, time)
        return pm.fromTf(pose)

    def getAllLinksTf(self, base_frame, time=None, timeout_s=1.0):
        """!
        Lookup transformations for all links.

        @param base_frame   string: name of base frame
        @param time         rospy.Time: time at which transform should be interpolated
        @param timeout_s    float: timeout in seconds
        @return dictionary {string:PyKDL.Frame} Returns dictionary that maps link names
            to their poses wrt. the base frame.
        @exception tf2_ros.TransformException when the transform for the specified time
            is not avaible during the timeout_s.
        """
        if time == None:
            time = rospy.Time.now()
        result = {}
        for l in self._all_links:
            try:
                result[l.name] = self._getKDLtf(base_frame, l.name, time, timeout_s)
            except:
                result[l.name] = None
        return result

    def getTf(self, frame_from, frame_to, time=None, timeout_s=1.0):
        """!
        Lookup tf transform and convert it to PyKDL.Frame. Frame names can be the full names
        or the following simplified names can be used:
         - 'Wo' - world frame ('world')
         - 'B' - robot base frame ('torso_base')
         - 'Wr' - right wrist ('right_arm_7_link')
         - 'Wl' - left wrist ('left_arm_7_link')
         - 'Gr' - right grip frame ('right_HandGripLink')
         - 'Gl' - left grip frame ('left_HandGripLink')
         - 'Tr' - right tool frame ('right_arm_tool')
         - 'Tl' - left tool frame ('left_arm_tool')
         - 'Fr00' - 'right_HandFingerOneKnuckleOneLink'
         - 'Fr01' - 'right_HandFingerOneKnuckleTwoLink'
         - 'Fr02' - 'right_HandFingerOneKnuckleThreeLink'
         - 'Fr10' - 'right_HandFingerTwoKnuckleOneLink'
         - 'Fr11' - 'right_HandFingerTwoKnuckleTwoLink'
         - 'Fr12' - 'right_HandFingerTwoKnuckleThreeLink'
         - 'Fr21' - 'right_HandFingerThreeKnuckleTwoLink'
         - 'Fr22' - 'right_HandFingerThreeKnuckleThreeLink'
         - 'Fl00' - 'left_HandFingerOneKnuckleOneLink'
         - 'Fl01' - 'left_HandFingerOneKnuckleTwoLink'
         - 'Fl02' - 'left_HandFingerOneKnuckleThreeLink'
         - 'Fl10' - 'left_HandFingerTwoKnuckleOneLink'
         - 'Fl11' - 'left_HandFingerTwoKnuckleTwoLink'
         - 'Fl12' - 'left_HandFingerTwoKnuckleThreeLink'
         - 'Fl21' - 'left_HandFingerThreeKnuckleTwoLink'
         - 'Fl22' - 'left_HandFingerThreeKnuckleThreeLink'

        @param frame_from   string: simplified name of base frame
        @param frame_to     string: simplified name of target frame
        @param time         rospy.Time: time at which transform should be interpolated
        @param timeout_s    float: timeout in seconds
        @return PyKDL.Frame transformation from base frame to target frame.
        @exception tf2_ros.TransformException when the transform for the specified time
            is not avaible during the timeout_s.
        """
        if time == None:
            time = rospy.Time.now()
        if frame_from in self._frames:
            frame_from_name = self._frames[frame_from]
        else:
            frame_from_name = frame_from

        if frame_to in self._frames:
            frame_to_name = self._frames[frame_to]
        else:
            frame_to_name = frame_to

        return self._getKDLtf( frame_from_name, frame_to_name, time, timeout_s )

