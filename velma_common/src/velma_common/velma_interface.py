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
import tf2_ros
import math
import time
import copy
import threading
from functools import partial
import PyKDL

from std_msgs.msg import Int32
from geometry_msgs.msg import Wrench, Vector3, Twist
from sensor_msgs.msg import JointState
from barrett_hand_msgs.msg import *
from barrett_hand_action_msgs.msg import *
from cartesian_trajectory_msgs.msg import *
from motor_action_msgs.msg import *
from grasped_action_msgs.msg import *
from identification_action_msgs.msg import *
from behavior_switch_action_msgs.msg import BehaviorSwitchAction, BehaviorSwitchGoal
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult,\
    FollowJointTrajectoryGoal, JointTolerance
from velma_look_at_action_msgs.msg import LookAtAction, LookAtGoal, LookAtResult
import diagnostic_msgs.msg
import tf_conversions.posemath as pm

import subsystem_common

import xml.dom.minidom as minidom

# Do not use rospy.sleep(), because it hangs when simulation is stopped or terminated.

def getSymmetricalJointName(joint_name):
    """!
    For joint names that refer to joints that are symmetrical (*left* or *right*), get the name
    of the another joint.

    @param joint_name string: name of joint.

    @return name of the joint symmetrical to joint_name, or None, if such does not exist.
    """
    if joint_name.startswith('left'):
        return 'right' + joint_name[4:]
    elif joint_name.startswith('right'):
        return 'left' + joint_name[5:]
    return None

def symmetricalConfiguration(q_map):
    """!
    Get configuration based on the input configuration such that all joint positions are symmetrical.

    @param q_map Dictionary: configuration {joint_name:joint_position}.

    @return Dictionary with symmetrical configuration.
    """
    result = {}
    for joint_name in q_map:
        assert not joint_name in result
        result[joint_name] = q_map[joint_name]
        sym_joint_name = getSymmetricalJointName(joint_name)
        if not sym_joint_name is None:
            assert not sym_joint_name in result
            result[sym_joint_name] = -q_map[joint_name]
    return result

def isConfigurationClose(q_map1, q_map2, tolerance=0.1, allow_subset=False):
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
        if allow_subset and (not joint_name in q_map1 or not joint_name in q_map2):
            continue
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

def splitTrajectory(joint_trajectory, max_traj_len):
    """!
    Split a long trajectory into a number of shorter trajectories.

    @param joint_trajectory         trajectory_msgs.msg.JointTrajectory: A joint trajectory to be splitted.
    @param max_traj_len             int: Maximum number of nodes in the result trajectory.
    @return Returns list of trajectory_msgs.msg.JointTrajectory -- the input trajectory split
    into a number of trajectories of length not greater than max_traj_len.
    """
    assert isinstance(joint_trajectory, JointTrajectory)
    if len(joint_trajectory.points) > max_traj_len:
        result = []
        #for idx in range(0, len(traj.trajectory.joint_trajectory.points), max_traj_len):
        idx = 0
        new_traj = JointTrajectory()
        new_traj.header = joint_trajectory.header
        new_traj.joint_names = joint_trajectory.joint_names
        time_base = rospy.Duration(0.0)
        for idx in range(len(joint_trajectory.points)):
            point = copy.copy( joint_trajectory.points[idx] )
            point.time_from_start = point.time_from_start - time_base
            new_traj.points.append( point )
            if len(new_traj.points) >= max_traj_len:
                result.append( new_traj )
                new_traj = JointTrajectory()
                new_traj.header = joint_trajectory.header
                new_traj.joint_names = joint_trajectory.joint_names
                point = copy.copy( joint_trajectory.points[idx] )
                time_base = point.time_from_start - rospy.Duration(0.5)
                point.time_from_start = point.time_from_start - time_base
                new_traj.points.append( point )

        if len(new_traj.points) > 0:
            result.append( new_traj )

        return result
    # else
    return [joint_trajectory]

# TODO: implement error handling using this mechanism
# class Result:
#     def __init__(self, generated_by, result_code, result_str, is_error):
#         assert generated_by in ('move_jnt', 'move_cart', 'move_head', 'move_hand')
#         assert isinstance(result_code, int)
#         assert isinstance(result_str, str)
#         assert isinstance(is_error, bool)
#         self.__generated_by = generated_by
#         self.__result_code = result_code
#         self.__result_str = result_str
#         self.__is_error = is_error

#     def getGeneratedBy(self):
#         return self.__generated_by

#     def getCode(self):
#         return self.__result_code

#     def getStr(self):
#         return self.__result_str

#     def isError(self):
#         return self.__is_error

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
        'wrist_right':'right_arm_7_link',
        'Wl':'left_arm_7_link',
        'wrist_left':'left_arm_7_link',
        'Er':'right_arm_7_link',
        'El':'left_arm_7_link',
        'Wright':'right_arm_7_link',
        'Wleft':'left_arm_7_link',
        'Gr':'right_HandGripLink',
        'grip_right':'right_HandGripLink',
        'Gl':'left_HandGripLink',
        'grip_left':'left_HandGripLink',
        'Pr':'right_HandPalmLink',
        'Pl':'left_HandPalmLink',
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
        CartImpResult.UNKNOWN_ERROR:'UNKNOWN_ERROR' }

    _joint_trajectory_result_names = {
        FollowJointTrajectoryResult.SUCCESSFUL:"SUCCESSFUL",
        FollowJointTrajectoryResult.INVALID_GOAL:"INVALID_GOAL",
        FollowJointTrajectoryResult.INVALID_JOINTS:"INVALID_JOINTS",
        FollowJointTrajectoryResult.OLD_HEADER_TIMESTAMP:"OLD_HEADER_TIMESTAMP",
        FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:"PATH_TOLERANCE_VIOLATED",
        FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:"GOAL_TOLERANCE_VIOLATED" }

    _look_at_result_names = {
        LookAtResult.SUCCESSFUL:'SUCCESSFUL',
        LookAtResult.OUT_OF_RANGE:'OUT_OF_RANGE',
        LookAtResult.WRONG_BEHAVIOUR:'WRONG_BEHAVIOUR',
        LookAtResult.MOTION_FAILED:'MOTION_FAILED' }

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

    _joint_groups = {
        "impedance_joints":['torso_0_joint', 'right_arm_0_joint', 'right_arm_1_joint',
            'right_arm_2_joint', 'right_arm_3_joint', 'right_arm_4_joint', 'right_arm_5_joint',
            'right_arm_6_joint', 'left_arm_0_joint', 'left_arm_1_joint', 'left_arm_2_joint',
            'left_arm_3_joint', 'left_arm_4_joint', 'left_arm_5_joint', 'left_arm_6_joint'],
        "right_arm":['right_arm_0_joint', 'right_arm_1_joint', 'right_arm_2_joint',
            'right_arm_3_joint', 'right_arm_4_joint', 'right_arm_5_joint', 'right_arm_6_joint'],
        "right_arm_torso":['torso_0_joint', 'right_arm_0_joint', 'right_arm_1_joint',
            'right_arm_2_joint', 'right_arm_3_joint', 'right_arm_4_joint', 'right_arm_5_joint',
            'right_arm_6_joint'],        
        "left_arm":['left_arm_0_joint', 'left_arm_1_joint', 'left_arm_2_joint', 'left_arm_3_joint',
            'left_arm_4_joint', 'left_arm_5_joint', 'left_arm_6_joint'],
        "left_arm_torso":['torso_0_joint', 'left_arm_0_joint', 'left_arm_1_joint',
            'left_arm_2_joint', 'left_arm_3_joint', 'left_arm_4_joint', 'left_arm_5_joint',
            'left_arm_6_joint'],
        "head":["head_pan_joint", "head_tilt_joint"]
        }

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
            js_pos = {}
            for joint_idx, joint_name in enumerate(data.name):
                js_pos[joint_name] = data.position[joint_idx]

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

    def __isActionConnected(self, action_name):
        self.dbgPrint('__isActionConnected: "{}"'.format(action_name))

        with self.__action_is_connected_lock:
            if action_name in self.__action_is_connected:
                is_connected = self.__action_is_connected[action_name]
            else:
                is_connected = False
        return is_connected

    def __connectAction(self, action_name, action, timeout_s):
        self.dbgPrint('__connectAction: wait for server "{}"'.format(action_name))
        is_connected = action.wait_for_server(rospy.Duration(timeout_s))
        self.dbgPrint('__connectAction: "{}", connected: {}'.format(action_name, is_connected))
        with self.__action_is_connected_lock:
            self.__action_is_connected[action_name] = is_connected

    def __getDiagInfo(self, timeout_s):
        self.__received_diag_info = False
        time_start = time.time()
        while not rospy.is_shutdown():
            try:
                diag = self.getCoreCsDiag()
                if len(diag.history) > 0:
                    self.__received_diag_info = True
                    return
            except:
                pass
            time.sleep(0.1)
            time_now = time.time()
            if timeout_s and (time_now-time_start) > timeout_s:
                break

    def __isInitialized(self):
        for action_name in self.__action_map:
            if not self.__action_obligatory_map[action_name]:
                continue
            if not self.__isActionConnected(action_name):
                return False
        if not self.__received_diag_info:
            return False
        if self._js_pos_history_idx < 0:
            return False
        return True

    def waitForInit(self, timeout_s=None):
        """!
        Wait for the interface until it is initialized.

        @param timeout_s float: Timeout in seconds.

        @return True if the interface was succesfully initialized within timeout, False otherwise.
        """
        self.dbgPrint('Waiting for init')

        self.dbgPrint('Getting ROS parameters')
        if not self.__tryGetRosParams():
            self.dbgPrint('Could not get all ROS parameters')
            return False
        self.dbgPrint('Got all ROS parameters')

        time_start = time.time()
        if timeout_s is None:
            timeout_s = 10.0

        self.__action_is_connected_lock = threading.Lock()
        self.__action_is_connected = {}
        threads = []
        for action_name, action in self.__action_map.iteritems():
            self.dbgPrint('Running thread for connecting action "{}"'.format(action_name))

            t = threading.Thread(name='action-{}'.format(action_name),
            			target=self.__connectAction, args=(action_name, action, timeout_s))
            t.daemon = True  # thread dies with the program
            t.start()
            threads.append( (action_name, t) )

        t = threading.Thread(name='get-diag-info',
                        		target=self.__getDiagInfo, args=(timeout_s,))
        t.start()
        threads.append( (None, t) )

        for action_name, t in threads:
            self.dbgPrint('Joining thread for connecting action "{}"'.format(action_name))
            if action_name is None or self.__action_obligatory_map[action_name]:
                t.join()

        self.dbgPrint('Waiting for full initialization')
        while (time.time()-time_start) < timeout_s:
            if self.__isInitialized():
                break
            time.sleep(0.1)

        if not self.__isInitialized():
            print("ERROR: waitForInit: timeout")
            for action_name in self.__action_map:
                if not action_name in self.__action_is_connected:
                    print('    is_action_connected({}): information is missing'.format(action_name))
                else:
                    print('    is_action_connected({}): {}'.format(action_name,
                                                        self.__action_is_connected[action_name]))

            print('    joint_state_received: {}'.format(self._js_pos_history_idx >= 0))
            print('    received_diag_info: {}'.format(self.__received_diag_info))
            self.dbgPrint('Failed to initialize')
            return False
        self.dbgPrint('Initialized')
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
            time.sleep(0.1)
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

    def getCartImpJointLimits(self):
        """!
        Gets limits of joints of both arms and torso in cart_imp mode.
        Joints of neck and grippers are not included.
        The joints are used in impedance control (cart_imp mode).

        @return dictionary: Returns a dictionary {name:(lower_limit, upper_limit, [lower_limit, upper_limit)])} that maps joint name to
        a 2-tupe or 4-tupe with lower and upper joint limit.
        """
        return self._cimp_joint_limits_map

    def getHeadJointLimits(self):
        """!
        Gets limits of joints of neck.

        @return dictionary: Returns a dictionary {name:(lower_limit, upper_limit)} that maps joint name to
        a 2-tupe with lower and upper joint limit.
        """
        return self._head_joint_limits

    def __tryGetRosParams(self):
        try:
            if self._body_joint_names is None:
                self.dbgPrint('Getting ROS parameter: {}/JntImpAction/joint_names'.format(self._task_cs_name))
                self._body_joint_names = rospy.get_param(self._task_cs_name+"/JntImpAction/joint_names")
                if len(self._body_joint_names) != 15:
                    raise RuntimeError("Wrong number of joints")

            if self._body_joint_limits is None:
                # body_joint_lower_limits = rospy.get_param(self._task_cs_name+"/JntImpAction/lower_limits")
                # body_joint_upper_limits = rospy.get_param(self._task_cs_name+"/JntImpAction/upper_limits")
                # if not body_joint_lower_limits is None and not body_joint_upper_limits is None:
                #     self._body_joint_limits = {}
                #     for i in range(len(self._body_joint_names)):
                #         self._body_joint_limits[self._body_joint_names[i]] = (body_joint_lower_limits[i], body_joint_upper_limits[i])

                # TODO: read limits from URDF
                self._body_joint_limits = {
                    "torso_0_joint": (-1.57, 1.57),
                    "right_arm_0_joint": (-2.96706, 2.96706),
                    "right_arm_1_joint": (-2.0944, 2.0944),
                    "right_arm_2_joint": (-2.96706, 2.96706),
                    "right_arm_3_joint": (-2.0944, 2.0944),
                    "right_arm_4_joint": (-2.96706, 2.96706),
                    "right_arm_5_joint": (-2.0944, 2.0944),
                    "right_arm_6_joint": (-2.96706, 2.96706),
                    "left_arm_0_joint": (-2.96706, 2.96706),
                    "left_arm_1_joint": (-2.0944, 2.0944),
                    "left_arm_2_joint": (-2.96706, 2.96706),
                    "left_arm_3_joint": (-2.0944, 2.0944),
                    "left_arm_4_joint": (-2.96706, 2.96706),
                    "left_arm_5_joint": (-2.0944, 2.0944),
                    "left_arm_6_joint": (-2.96706, 2.96706),
                }

            if self._cimp_joint_limits_map is None:
                self.dbgPrint('Getting ROS parameter: {}/JntLimit/joint_names'.format(self._core_cs_name))
                joint_names = rospy.get_param(self._core_cs_name+"/JntLimit/joint_names")
                if not joint_names is None:
                    self._cimp_joint_limits_map = {}
                    for idx, joint_name in enumerate(joint_names):
                        self.dbgPrint('Getting ROS parameter: {}/JntLimit/limits_{}'.format(self._core_cs_name, idx))
                        jnt_limits = rospy.get_param('{}/JntLimit/limits_{}'.format(self._core_cs_name, idx))
                        self._cimp_joint_limits_map[joint_name] = []
                        for lim in jnt_limits:
                            self._cimp_joint_limits_map[joint_name].append( float(lim) )

            if self._head_joint_names is None:
                self.dbgPrint('Getting ROS parameter: {}/HeadAction/joint_names'.format(self._task_cs_name))
                self._head_joint_names = rospy.get_param(self._task_cs_name+"/HeadAction/joint_names")

            if self._head_joint_limits is None:
                # head_joint_lower_limits = rospy.get_param(self._task_cs_name+"/HeadAction/lower_limits")
                # head_joint_upper_limits = rospy.get_param(self._task_cs_name+"/HeadAction/upper_limits")
                # if not head_joint_lower_limits is None and not head_joint_upper_limits is None:
                #     self._head_joint_limits = {}
                #     for i in range(len(self._head_joint_names)):
                #         self._head_joint_limits[self._head_joint_names[i]] = (head_joint_lower_limits[i], head_joint_upper_limits[i])

                # TODO: read limits from URDF
                self._head_joint_limits = {
                    "head_pan_joint": (-1.57, 1.57),
                    "head_tilt_joint": (-1, 1.3),
                }

            if self._all_joint_names is None:
                self.dbgPrint('Getting ROS parameter: {}/JntPub/joint_names'.format(self._task_cs_name))
                self._all_joint_names = rospy.get_param(self._task_cs_name+"/JntPub/joint_names")

            if self._all_links is None:
                self._all_links = []

                self.dbgPrint('Getting ROS parameter: /robot_description')
                robot_description_xml = rospy.get_param("/robot_description")
                #print robot_description_xml

                self.dbgPrint('parsing robot_description')
                dom = minidom.parseString(robot_description_xml)
                robot = dom.getElementsByTagName("robot")
                if len(robot) != 1:
                    raise Exception("Could not parse robot_description xml: wrong number of 'robot' elements.")
                links = robot[0].getElementsByTagName("link")
                for l in links:
                    name = l.getAttribute("name")
                    if name == None:
                        raise Exception("Could not parse robot_description xml: link element has no name.")

                    self.dbgPrint('parsing link "{}"'.format(name))

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

            if self.__wcc_l_poly is None:
                self.dbgPrint('Getting ROS parameter: {}/wcc_l/constraint_polygon'.format(self._core_cs_name))
                self.__wcc_l_poly = rospy.get_param(self._core_cs_name+"/wcc_l/constraint_polygon")
            if self.__wcc_r_poly is None:
                self.dbgPrint('Getting ROS parameter: {}/wcc_r/constraint_polygon'.format(self._core_cs_name))
                self.__wcc_r_poly = rospy.get_param(self._core_cs_name+"/wcc_r/constraint_polygon")
        except Exception as e:
            self.dbgPrint('Exception on getting ROS parameters: {}'.format(e))

            pass

        if self._body_joint_names is None or\
                self._body_joint_limits is None or\
                self._cimp_joint_limits_map is None or\
                self._head_joint_names is None or\
                self._head_joint_limits is None or\
                self._all_joint_names is None or\
                self._all_links is None or\
                self.__wcc_l_poly is None or\
                self.__wcc_r_poly is None:
            #self.dbgPrint('Getting ROS parameter: {}/wcc_r/constraint_polygon'.format(_core_cs_name))

            return False
        return True

    def dbgPrint(self, s):
        if self.__debug:
            print('VelmaInterface: {}'.format(s))
            time.sleep(0.01)

    def __init__(self, debug=False):
        """!
        Initialization of the interface.
        """
        self.__debug = debug
        self.dbgPrint('Running initializer')

        self._body_joint_names = None
        self._body_joint_limits = None
        self._cimp_joint_limits_map = None
        self._head_joint_names = None
        self._head_joint_limits = None
        self._all_joint_names = None
        self._all_links = None
        self.__wcc_l_poly = None
        self.__wcc_r_poly = None

        self.dbgPrint('Creating tf listener')
        self._listener = tf.TransformListener()

        # read the joint information from the ROS parameter server

        self._js_pos_history = []
        for i in range(200):
            self._js_pos_history.append( None )
        self._js_pos_history_idx = -1

        self._joint_states_lock = threading.Lock()

        # The map of ROS actions:
        # self.__action_map[name] = (is_obligatory, action_client)
        self.__action_map = {
        	'jimp':
                actionlib.SimpleActionClient(
                "/velma_task_cs_ros_interface/spline_trajectory_action_joint",
                FollowJointTrajectoryAction),
        	'head':
                actionlib.SimpleActionClient(
                "/velma_task_cs_ros_interface/head_spline_trajectory_action_joint",
                FollowJointTrajectoryAction),
        	'relax':
                actionlib.SimpleActionClient(
                "/velma_task_cs_ros_interface/relax_action",
                BehaviorSwitchAction),
        	'grasped_right':
                actionlib.SimpleActionClient(
                "/velma_task_cs_ros_interface/right_arm/grasped_action",
                GraspedAction),
        	'grasped_left':
                actionlib.SimpleActionClient(
                "/velma_task_cs_ros_interface/left_arm/grasped_action",
                GraspedAction),
        	'identification_right':
                actionlib.SimpleActionClient(
                "/velma_task_cs_ros_interface/right_arm/identification_action",
                IdentificationAction),
        	'identification_left':
                actionlib.SimpleActionClient(
                "/velma_task_cs_ros_interface/left_arm/identification_action",
                IdentificationAction),
            'hp':
                actionlib.SimpleActionClient(
                "/velma_task_cs_ros_interface/motors/hp",
                MotorAction),
            'ht':
                actionlib.SimpleActionClient(
                "/velma_task_cs_ros_interface/motors/ht",
                MotorAction),
            't':
                actionlib.SimpleActionClient(
                "/velma_task_cs_ros_interface/motors/t",
                MotorAction),
            'hand_right':
                actionlib.SimpleActionClient(
                "/velma_task_cs_ros_interface/right_hand/move_hand",
                BHMoveAction),
            'hand_left':
                actionlib.SimpleActionClient(
                "/velma_task_cs_ros_interface/left_hand/move_hand",
                BHMoveAction),
            'cimp_right':
                actionlib.SimpleActionClient(
                "/velma_task_cs_ros_interface/right_arm/cartesian_trajectory",
                CartImpAction),
            'cimp_left':
                actionlib.SimpleActionClient(
                "/velma_task_cs_ros_interface/left_arm/cartesian_trajectory",
                CartImpAction),
            'look_at':
                actionlib.SimpleActionClient(
                '/velma_look_at_action',
                LookAtAction),
        }
        self.__action_obligatory_map  = {}
        for name in self.__action_map:
            self.__action_obligatory_map[name] = True
        self.__action_obligatory_map['look_at'] = False

        time.sleep(1.0)

        self.dbgPrint('Creating publisher /velma_task_cs_ros_interface/allow_hands_col_in')
        self.__pub_allow_hands_col = rospy.Publisher('/velma_task_cs_ros_interface/allow_hands_col_in', Int32, queue_size=10)

        #self.wrench_tab = []
        #self.wrench_tab_index = 0
        #self.wrench_tab_len = 4000
        #for i in range(0,self.wrench_tab_len):
        #    self.wrench_tab.append( Wrench(Vector3(), Vector3()) )

        self.dbgPrint('Creating subscriber /joint_states')
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

        for topic_name, topic_type in subscribed_topics_list:
            self.dbgPrint('Creating subscriber {}'.format(topic_name))
            self._subscribed_topics[topic_name] = VelmaInterface.SubscribedTopic(
                                                        topic_name, topic_type, debug=self.__debug)

        self.dbgPrint('Initializer function is done')

    class SubscribedTopic:
        """!
        Class used for subscription for various ROS topics from the VelmaInterface class.

        """

        def dbgPrint(self, s):
            if self.__debug:
                print('SubscribedTopic({}): {}'.format(self.__topic_name, s))
                time.sleep(0.1)

        def __init__(self, topic_name, topic_type, debug=False):
            self.__debug = debug
            self.__mutex = threading.Lock()
            self.__topic_name = topic_name
            self.__data = None
            self.sub = rospy.Subscriber(self.__topic_name, topic_type, self.__callback)

        def __callback(self, data):
            #self.dbgPrint('new data')
            with self.__mutex:
                self.__data = data

        def getData(self, timeout_s=None):
            """!
            Get the newest topic data.

            @param timeout_s float: timeout in seconds or None for no timeout. Default is None (no
            timeout, return immediately).

            @return Returns the newest data read on topic if it is available, or None otherwise.

            """
            #self.dbgPrint('get data')

            if not timeout_s is None:
                end_time = rospy.Time.now() + rospy.Duration(timeout_s)
            result = None
            while not rospy.is_shutdown():
                with self.__mutex:
                    if self.__data:
                        result = copy.copy( self.__data )
                if not result is None or timeout_s is None or rospy.Time.now() >= end_time:
                    return result
                try:
                    time.sleep(0.1)
                except:
                    return None
            return None

    def _getSubsystemDiag(self, subsystem_name, timeout_s=None):
        data = self._getTopicData(subsystem_name + "/diag", timeout_s=timeout_s)
        if data == None:
            return None
        for v in data.status[1].values:
            if v.key == "master_component":
                mcd = subsystem_common.parseMasterComponentDiag(v.value)
                return mcd
        return None

    class CoreVeBodyDiag(subsystem_common.SubsystemDiag):
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

            self.__current_state = parent.history[0].state_name
            assert (self.__current_state == "idle" or self.__current_state == "transp" or
                self.__current_state == "transp_st" or self.__current_state == "safe" or
                self.__current_state == "safe_st" or self.__current_state == "safe_st_ok")

        def getCurrentStateName(self):
            return self.__current_state

        def getCurrentStateModeName(self):
            if self.__current_state == 'idle':
                return 'idle'
            elif self.__current_state == 'safe' or self.__current_state == 'safe_st' or\
                    self.__current_state == 'safe_st_ok':
                return 'safe / safe_st / safe_st_ok'
            elif self.__current_state == 'transp' or self.__current_state == 'transp_st':
                return 'transp / transp_st'
            else:
                raise Exception()

    def getCoreVeDiag(self, timeout_s=None):
        """!
        Get diagnostic information for core VE.

        @return Returns object of type subsystem_common.SubsystemDiag, with
            diagnostic information about subsystem.
        """
        return self.CoreVeBodyDiag(self._getSubsystemDiag(self._core_ve_name, timeout_s=timeout_s))

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

            self.__current_state = parent.history[0].state_name
            assert (self.__current_state == "idle" or self.__current_state == "safe" or
                self.__current_state == "cart_imp" or self.__current_state == "jnt_imp" or
                self.__current_state == "relax")

        def getCurrentStateName(self):
            return self.__current_state

        def inStateIdle(self):
            """!
            Information about current state.
            @return True if the subsystem is in 'idle' state, False otherwise.
            """
            return self.__current_state == "idle"

        def inStateSafe(self):
            """!
            Information about current state.
            @return True if the subsystem is in 'safe' state, False otherwise.
            """
            return self.__current_state == "safe"

        def inStateCartImp(self):
            """!
            Information about current state.
            @return True if the subsystem is in 'cart_imp' state, False otherwise.
            """
            return self.__current_state == "cart_imp"

        def inStateJntImp(self):
            """!
            Information about current state.
            @return True if the subsystem is in 'jnt_imp' state, False otherwise.
            """
            return self.__current_state == "jnt_imp"

        def inStateRelax(self):
            """!
            Information about current state.
            @return True if the subsystem is in 'relax' state, False otherwise.
            """
            return self.__current_state == "relax"

        def isSafeReasonSelfCol(self):
            """!
            Information about reason for entering 'safe' state.
            @return True if the subsystem entered 'safe' state due to possibility of self-collision,
                False otherwise.
            @exception AssertionError   Raised when current state is not 'safe'.
            @exception KeyError         Raised when 'inSelfCollision' predicate cannot be obtained.
            """
            assert (self.__current_state == "safe" and self.history[0].state_name == "safe")
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
            assert (self.__current_state == "safe" and self.history[0].state_name == "safe" and len(self.history) >= 2)

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

    def getCoreCsDiag(self, timeout_s=None):
        """!
        Get diagnostic information for core CS.

        @return Returns object of type VelmaInterface.CoreCsDiag, with
            diagnostic information about control subsystem.
        """
        return self.CoreCsDiag(self._getSubsystemDiag(self._core_cs_name, timeout_s=timeout_s))

    # Private method
    def _getTopicData(self, topic, timeout_s=None):
        return self._subscribed_topics[topic].getData(timeout_s=timeout_s)

    def allowHandsCollisions(self):
        """!
        Allow self-collisions of hands.

        @return None
        """
        data = Int32()
        data.data = 1
        self.__pub_allow_hands_col.publish( data )

    def disallowHandsCollisions(self):
        """!
        Disallow self-collisions of hands.

        @return None
        """
        data = Int32()
        data.data = 0
        self.__pub_allow_hands_col.publish( data )

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

    def getLeftWccPolygon(self):
        """!
        Get Polygon that defines wrist collision constraints for joints 5 and 6 of the left arm.

        @return list: Returns wrist collision constraint polygon as [x0, y0, x1, y1, x2, y2, ...].
        """
        return self.__wcc_l_poly

    def getRightWccPolygon(self):
        """!
        Get Polygon that defines wrist collision constraints for joints 5 and 6 of the right arm.

        @return list: Returns wrist collision constraint polygon as [x0, y0, x1, y1, x2, y2, ...].
        """
        return self.__wcc_r_poly

    # Private method
    def _action_right_cart_traj_feedback_cb(self, feedback):
        self._action_right_cart_traj_feedback = copy.deepcopy(feedback)

    # Private method
    def _wrenchKDLtoROS(self, wrKDL):
        return geometry_msgs.msg.Wrench(Vector3( wrKDL.force.x(), wrKDL.force.y(), wrKDL.force.z() ), Vector3( wrKDL.torque.x(), wrKDL.torque.y(), wrKDL.torque.z() ))

    # Private method
    def _wrenchROStoKDL(self, wrROS):
        return PyKDL.Wrench( PyKDL.Vector(wrROS.force.x, wrROS.force.y, wrROS.force.z), PyKDL.Vector(wrROS.torque.x, wrROS.torque.y, wrROS.torque.z) )

    def switchToRelaxBehavior(self):
        """!
        Switches the robot to relax behavior.
        """
        goal = BehaviorSwitchGoal()
        goal.command = 1
        self.__action_map['relax'].send_goal(goal)

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
        self.__action_map[motor].send_goal(goal)

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
        self.__action_map[motor].send_goal(goal)

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

        if not self.__action_map[motor].wait_for_result(timeout=rospy.Duration(timeout_s)):
            return None
        result = self.__action_map[motor].get_result()

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

    def getCartImpMvTime(self, side, T_B_T, max_vel_lin, max_vel_rot, current_T_B_T=None):
        """!
        Calculate movement time for a given destination pose of end effector tool, given maximum
        allowed linear and rotational velocities.

        @param side string: side: 'left' or 'right'.
        @param T_B_T PyKDL.Frame: destination pose.
        @param max_vel_lin float: maximum linear velocity.
        @param max_vel_rot float: maximum rotational velocity.
        @param current_T_B_T PyKDL.Frame: (optional) current pose. If ommited, the current pose
        from tf is considered.

        @return Returns float: movement time in seconds.

        @exception AssertionError Raised when prefix is neither 'left' nor 'right'.
        """
        if current_T_B_T is None:
            if side == 'left':
                current_T_B_T = self.getTf('B', 'Tl', time=None, timeout_s=0.1)
                if current_T_B_T is None:
                    current_T_B_T = self.getTf('B', 'Wl', time=None, timeout_s=0.1)
            elif side == 'right':
                current_T_B_T = self.getTf('B', 'Tr', time=None, timeout_s=0.1)
                if current_T_B_T is None:
                    current_T_B_T = self.getTf('B', 'Wr', time=None, timeout_s=0.1)
            else:
                raise Exception('Wrong side: "{}"'.format(side))
        twist = PyKDL.diff(current_T_B_T, T_B_T, 1.0)
        return max( twist.vel.Norm()/max_vel_lin, twist.rot.Norm()/max_vel_rot )

    def moveCartImp(self, prefix, pose_list_T_B_Td, pose_times, tool_list_T_W_T, tool_times, imp_list, imp_times, max_wrench, start_time=0.01, stamp=None, damping=PyKDL.Wrench(PyKDL.Vector(0.35, 0.35, 0.35),PyKDL.Vector(0.35, 0.35, 0.35)), path_tol=None):
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
        self.__action_map['cimp_'+prefix].send_goal(action_trajectory_goal)

        return True

    def moveCartImpRight(self, pose_list_T_B_Td, pose_times, tool_list_T_W_T, tool_times, imp_list, imp_times, max_wrench, start_time=0.01, stamp=None, damping=PyKDL.Wrench(PyKDL.Vector(0.35, 0.35, 0.35),PyKDL.Vector(0.35, 0.35, 0.35)), path_tol=None):
        """!
        Execute motion in cartesian impedance mode for the right end-effector.
        @see moveCartImp
        """
        return self.moveCartImp("right", pose_list_T_B_Td, pose_times, tool_list_T_W_T, tool_times, imp_list, imp_times, max_wrench, start_time=start_time, stamp=stamp, damping=damping, path_tol=path_tol)

    def moveCartImpLeft(self, pose_list_T_B_Td, pose_times, tool_list_T_W_T, tool_times, imp_list, imp_times, max_wrench, start_time=0.01, stamp=None, damping=PyKDL.Wrench(PyKDL.Vector(0.35, 0.35, 0.35),PyKDL.Vector(0.35, 0.35, 0.35)), path_tol=None):
        """!
        Execute motion in cartesian impedance mode for the left end-effector.
        @see moveCartImp
        """
        return self.moveCartImp("left", pose_list_T_B_Td, pose_times, tool_list_T_W_T, tool_times, imp_list, imp_times, max_wrench, start_time=start_time, stamp=stamp, damping=damping, path_tol=path_tol)

    def moveCartImpCurrentPos(self, side, start_time=0.2, stamp=None):
        """!
        Move right end-effector to current position. Switch core_cs to cart_imp mode.
        @return Returns True.
        """
        return self.moveCartImp(side, None, None, None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=start_time, stamp=stamp)

    def moveCartImpRightCurrentPos(self, start_time=0.2, stamp=None):
        """!
        Move right end-effector to current position. Switch core_cs to cart_imp mode.
        @return Returns True.
        """
        return self.moveCartImpCurrentPos('right', start_time=start_time, stamp=stamp)

    def moveCartImpLeftCurrentPos(self, start_time=0.2, stamp=None):
        """!
        Move left end-effector to current position. Switch core_cs to cart_imp mode.
        @return Returns True.
        """
        return self.moveCartImpCurrentPos('left', start_time=start_time, stamp=stamp)

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
        if not self.__action_map['cimp_'+prefix].wait_for_result(timeout=rospy.Duration(timeout_s)):
            return None
        result = self.__action_map['cimp_'+prefix].get_result()
        if result.error_code != 0:
            error_str = "UNKNOWN"
            if result.error_code in self._cartesian_trajectory_result_names:
                error_str = self._cartesian_trajectory_result_names[result.error_code]
            print "waitForEffector(" + prefix + "): action failed with error_code=" + str(result.error_code) + " (" + error_str + ")"

        self.waitForJointState( self.__action_map['cimp_'+prefix].gh.comm_state_machine.latest_result.header.stamp )

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



    def lookAt(self, point, frame_id='torso_base'):
        """!
        Execute look at motion of head.

        @return Returns True

        """

        assert self.__isActionConnected('look_at')

        goal = LookAtGoal()
        goal.frame_id = frame_id
        goal.target = geometry_msgs.msg.Point(point.x(), point.y(), point.z())
        self.__action_map['look_at'].send_goal(goal)

        return True

    def cancelLookAt(self):
        """!
        Cancel look at motion of head.

        @return Returns True.

        """

        assert self.__isActionConnected('look_at')

    def isLookAtConnected(self):
        """!
        Check if look at action is connected.

        @return True if look at action is connected, False otherwise.

        """
        return self.__isActionConnected('look_at')

    def waitForLookAt(self, timeout_s=None):
        """!
        Wait for completion of look at motion.

        @param timeout_s    float: timeout in seconds.

        @return Returns error code.

        """

        assert self.__isActionConnected('look_at')

        if timeout_s == None:
            timeout_s = 0
        if not self.__action_map['look_at'].wait_for_result(timeout=rospy.Duration(timeout_s)):
            return None
        result = self.__action_map['look_at'].get_result()
        if result.error_code != 0:
            error_str = "UNKNOWN"
            if result.error_code in self._look_at_result_names:
                error_str = self._look_at_result_names[result.error_code]
            print('waitForLookAt(): action failed with error_code={} ({})'.format(
                                                                    result.error_code, error_str))
        self.waitForJointState( self.__action_map['look_at'].gh.comm_state_machine.latest_result.header.stamp )
        return result.error_code

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

    @staticmethod
    def getJointGroup(group_name):
        """!
        Get names of all joints in group.
        @param group_name   string: name of group.
        @return Returns list of names of joints in group.
        """
        return VelmaInterface._joint_groups[group_name]

    def moveJointTrajAndWait(self, traj, start_time=0.2, stamp=None, position_tol=5.0/180.0 * math.pi, velocity_tol=5.0/180.0*math.pi):
        """!
        Execute joint space trajectory in joint impedance mode. Trajectories of any length are allowed.
        This method blocks the executing thread until trajectory is completed, or error occures.
        @param traj         trajectory_msgs.msg.JointTrajectory: joint trajectory.
        @param start_time   float: relative start time.
        @param stamp        rospy.Time: absolute start time.
        @position_tol       float: position tolerance.
        @velocity_tol       float: velocity tolerance.
        @return Returns True if trajectory was executed.
        """

        q_end_traj = {}
        for q_idx, joint_name in enumerate(traj.joint_names):
            q_end_traj[joint_name] = traj.points[-1].positions[q_idx]

        traj_list = splitTrajectory(traj, self.maxJointTrajLen())
        for traj_idx, traj_part in enumerate(traj_list):
            print('Executing trajectory {}...'.format(traj_idx))
            if not self.moveJointTraj(traj_part, start_time=0.5,
                        position_tol=position_tol, velocity_tol=velocity_tol):
                exitError(5)
            if self.waitForJoint() != 0:
                print('VelmaInterface.moveJointTraj(): The trajectory could not be completed')
                return False
        rospy.sleep(0.5)
        js = self.getLastJointState()
        if not isConfigurationClose(q_end_traj, js[1], tolerance=position_tol, allow_subset=True):
            print('VelmaInterface.moveJointTraj(): the goal configuration could not be reached')
            return False
        # else:
        return True

    def moveJointTraj(self, traj, start_time=0.2, stamp=None, position_tol=5.0/180.0 * math.pi, velocity_tol=5.0/180.0*math.pi):
        """!
        Execute joint space trajectory in joint impedance mode. Trajectories of length up to 50 nodes are allowed.
        This method does not block the executing thread.
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
        self.__action_map['jimp'].send_goal(goal)
        return True

    def calculateJointTrajTime(self, q_map_start, traj_in, max_vel):
        if q_map_start is None:
            q_map_start = self.getLastJointState()[1]

        pt_idx = 0
        q_map_prev = q_map_start

        #print('VelmaInterface.calculateJointTrajTime()')
        traj_out = JointTrajectory()
        traj_out.header = traj_in.header 
        traj_out.joint_names = traj_in.joint_names
        time_from_start = 0.0
        for pt_idx in range(0, len(traj_in.points)):
            q_map = {}
            for joint_idx, joint_name in enumerate(traj_in.joint_names):
                q_map[joint_name] = traj_in.points[pt_idx].positions[joint_idx]
            time_from_start = time_from_start + VelmaInterface.getJntImpMovementTime2(q_map_prev, q_map, max_vel)
            #print('  idx: {}, time_from_start: {}'.format(pt_idx, time_from_start))
            q_map_prev = q_map
            point_out = JointTrajectoryPoint()
            point_out.positions = traj_in.points[pt_idx].positions
            point_out.velocities = traj_in.points[pt_idx].velocities
            point_out.time_from_start = rospy.Duration(time_from_start)
            traj_out.points.append( point_out )
        return traj_out

    def getJntImpMovementTime(self, q_dest_map, max_vel):
        js = self.getLastJointState()[1]
        return VelmaInterface.getJntImpMovementTime2(js, q_dest_map, max_vel)

    @staticmethod
    def getJntImpMovementTime2(q_dest_map_1, q_dest_map_2, max_vel):
        max_dist = 0.0
        for joint_name in q_dest_map_1:
            if not joint_name in q_dest_map_2:
                continue
            dist = abs(q_dest_map_1[joint_name] - q_dest_map_2[joint_name])
            if joint_name == 'torso_0_joint':
                dist = dist * 3.0
            max_dist = max(max_dist, dist)
        return max_dist / max_vel

    def moveJoint(self, q_dest_map, time, max_vel=None, start_time=0.2, stamp=None, position_tol=5.0/180.0 * math.pi, velocity_tol=5.0/180.0*math.pi):
        """!
        Execute simple joint space motion in joint impedance mode.
        @param q_dest_map   dictionary: dictionary {name:position} of goal configuration
        @param start_time   float: relative start time.
        @param stamp        rospy.Time: absolute start time.
        @position_tol       float: position tolerance.
        @velocity_tol       float: velocity tolerance.
        @return Returns True.
        """
        if time is None:
            assert not max_vel is None
            time = max(0.1, self.getJntImpMovementTime(q_dest_map, max_vel))
            print('moveJoint calculated time: {}'.format(time))
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

    def waitForJoint(self, timeout_s=None):
        """!
        Wait for joint space movement to complete.
        @return Returns error code.
        """
        if timeout_s == None:
            timeout_s = 0
        if not self.__action_map['jimp'].wait_for_result(timeout=rospy.Duration(timeout_s)):
            return None
        result = self.__action_map['jimp'].get_result()
        if result.error_code != 0:
            error_str = "UNKNOWN"
            if result.error_code in self._joint_trajectory_result_names:
                error_str = self._joint_trajectory_result_names[result.error_code]
            print('waitForJoint(): action failed with error_code={} ({})'.format(
                                                                    result.error_code, error_str))

        self.waitForJointState( self.__action_map['jimp'].gh.comm_state_machine.latest_result.header.stamp )

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
        self.__action_map['head'].cancel_goal()
        self.__action_map['head'].send_goal(goal)
        return True

    def __getHeadMovementTime(self, q_dest, max_vel):
        js = self.getLastJointState()[1]
        dist1 = abs(q_dest[0] - js["head_pan_joint"])
        dist2 = abs(q_dest[1] - js["head_tilt_joint"])
        max_dist = max(dist1, dist2)
        return max_dist / max_vel

    def moveHead(self, q_dest, time, max_vel=None, start_time=0.2, stamp=None, position_tol=5.0/180.0 * math.pi, velocity_tol=5.0/180.0*math.pi):
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

        if time is None:
            assert not max_vel is None
            time = max(0.1, self.__getHeadMovementTime(q_dest, max_vel))
            print('moveHead calculated time: {}'.format(time))

        traj = JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = q_dest
        pt.velocities = [0.0, 0.0]
        pt.time_from_start = rospy.Duration(time)
        traj.points.append(pt)
        return self.moveHeadTraj(traj, start_time=start_time, stamp=stamp, position_tol=position_tol, velocity_tol=velocity_tol)

    def waitForHead(self, timeout_s=None):
        """!
        Wait for head movement to complete.
        @return Returns error code.
        """
        if timeout_s == None:
            timeout_s = 0

        if not self.__action_map['head'].wait_for_result(timeout=rospy.Duration(timeout_s)):
            return None
        result = self.__action_map['head'].get_result()
        if result.error_code != 0:
            error_str = "UNKNOWN"
            if result.error_code in self._joint_trajectory_result_names:
                error_str = self._joint_trajectory_result_names[result.error_code]
            print('waitForHead(): action failed with error_code={} ({})'.format(
                                                                    result.error_code, error_str))

        self.waitForJointState( self.__action_map['head'].gh.comm_state_machine.latest_result.header.stamp )

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
        action_goal.hold = 1 if hold else 0
        self.__action_map['hand_'+prefix].send_goal(action_goal)

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
        self.__action_map['hand_'+prefix].send_goal(action_goal)

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

    def waitForHand(self, prefix, timeout_s=None):
        """!
        Wait for completion of hand movement.
        @param prefix       string: name of hand, either 'left' or 'right'
        @exception AssertionError when prefix is neither 'left' nor 'right'
        """
        assert (prefix == 'left' or prefix == 'right')

        if timeout_s == None:
            timeout_s = 0

        if not self.__action_map['hand_'+prefix].wait_for_result(timeout=rospy.Duration(timeout_s)):
            return None

        result = self.__action_map['hand_'+prefix].get_result()
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
        try:
            self._listener.waitForTransform(base_frame, frame, time, rospy.Duration(timeout_s))
            pose = self._listener.lookupTransform(base_frame, frame, time)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException,
                tf2_ros.TransformException):
            return None
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
         - 'Er' - same as 'Wr'
         - 'El' - same as 'Wl'
         - 'Gr' - right grip frame ('right_HandGripLink')
         - 'Gl' - left grip frame ('left_HandGripLink')
         - 'Pr' - right palm frame ('right_HandPalmLink')
         - 'Pl' - left palm frame ('left_HandPalmLink')
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

    def setGraspedFlag(self, side, status):
        """!
        Inform control system about grasped object.

        @param side string: Hand name, can be one of two values ('left' or 'right').        
        @param status bool: True when object is grasped and gravity compensation is active, False
        otherwise.

        @exception NameError: If side is not 'left' or 'right'.         
        """
        if side != 'left' and side != 'right':
            raise NameError('wrong side name: ' + str(side))

        goal = GraspedGoal()
        if status == True:
            goal.action = GraspedGoal.ACTION_OBJECT_GRASPED
        elif status == False:
            goal.action = GraspedGoal.ACTION_NOTHING_GRASPED

        self.__action_map['grasped_'+side].send_goal(goal)

        print "Manipulator:", side, "--> object_grasped_flag_status:", status 

        self.__action_map['grasped_'+side].wait_for_result(timeout=rospy.Duration(0))
        result = self.__action_map['grasped_'+side].get_result()
        error_code = result.error_code
        if error_code != 0:
            print "setGraspedFlag: action failed (error)"

    def sendIdentificationMeasurementCommand(self, side, command_index):
        """!
        Make measurement for gravity compensation. This identification action requires four
        measurements: two before and two after object is grasped.

        @param side string: Hand name, can be one of two values ('left' or 'right').
        @param command_index int: measurement index - one of the following values: 1 - the first
        measurement before the object is grasped, 2 - the second measurement before the object
        is grasped, 3 - the first measurement after the object is grasped (in the same pose
        as for measurement 1), 4 - the second measurement after the object is grasped (in the same
        pose as for measurement 2).

        @exception NameError: If side is not 'left' or 'right'.         
        """
        if side != 'left' and side != 'right':
            raise NameError('wrong side name: ' + str(side))

        goal = IdentificationGoal()
        if command_index == 1:
            goal.action = IdentificationGoal.ACTION_FIRST_MEASUREMENT_BEFORE_OBJECT_IS_GRASPED
        elif command_index == 2:
            goal.action = IdentificationGoal.ACTION_SECOND_MEASUREMENT_BEFORE_OBJECT_IS_GRASPED
        elif command_index == 3:
            goal.action = IdentificationGoal.ACTION_FIRST_MEASUREMENT_AFTER_OBJECT_IS_GRASPED            
        elif command_index == 4:
            goal.action = IdentificationGoal.ACTION_SECOND_MEASUREMENT_AFTER_OBJECT_IS_GRASPED

        self.__action_map['identification_'+side].send_goal(goal)

        print "Manipulator:", side, "--> identification_command:", command_index 

        self.__action_map['identification_'+side].wait_for_result(timeout=rospy.Duration(0))
        result = self.__action_map['identification_'+side].get_result()
        error_code = result.error_code
        if error_code != 0:
            print "sendIdentificationMeasurementCommand: action failed (error)"
