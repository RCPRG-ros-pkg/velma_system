"""
This file contains ROS-based Python interface for WUT Velma Robot and some helper functions.
"""

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
from motor_action_msgs.msg import *
from behavior_switch_action_msgs.msg import *
import actionlib
from trajectory_msgs.msg import *
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTolerance
import diagnostic_msgs.msg
import tf_conversions.posemath as pm

import subsystem_common.subsystem_diag as subsystem_diag

import xml.dom.minidom as minidom

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
            if can_break and self._allActionsConnected():
                break
            rospy.sleep(0.1)
            time_now = time.time()
            if timeout_s and (time_now-time_start) > timeout_s:
                print "ERROR: waitForInit: ", can_break, self._action_move_hand_client_connected["right"],\
                    self._action_move_hand_client_connected["left"], self._action_cart_traj_client_connected["right"],\
                    self._action_cart_traj_client_connected["left"], self._action_joint_traj_client_connected,\
                    self._action_head_joint_traj_client_connected, self._action_motor_client_connected,\
                    self._action_safe_col_client_connected
                return False
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

        #print "self._all_links", self._all_links
        #print "self._all_joint_names", self._all_joint_names

        self._js_pos_history = []
        for i in range(200):
            self._js_pos_history.append( None )
        self._js_pos_history_idx = -1

        # parameters
        #self.T_B_W = None
        #self.wrench_emergency_stop = False

        self._joint_states_lock = threading.Lock()

        self._action_move_hand_client_connected = {'right':False, 'left':False}
        self._action_cart_traj_client_connected = {'right':False, 'left':False}
        self._action_joint_traj_client_connected = False
        self._action_head_joint_traj_client_connected = False
        self._action_safe_col_client_connected = False

        self._action_motor_client_connected = {'hp':False, 'ht':False, 't':False}

        # cartesian wrist trajectory for right arm
        self._action_cart_traj_client = {
            'right':actionlib.SimpleActionClient("/right_arm/cartesian_trajectory", CartImpAction),
            'left':actionlib.SimpleActionClient("/left_arm/cartesian_trajectory", CartImpAction)
            }

        # joint trajectory for arms and torso
        self._action_joint_traj_client = actionlib.SimpleActionClient("/spline_trajectory_action_joint", FollowJointTrajectoryAction)

        # joint trajectory for head
        self._action_head_joint_traj_client = actionlib.SimpleActionClient("/head_spline_trajectory_action_joint", FollowJointTrajectoryAction)

        self._action_safe_col_client = actionlib.SimpleActionClient("/safe_col_action", BehaviorSwitchAction)

        # motor actions for head
        self._action_motor_client = {
            'hp':actionlib.SimpleActionClient("/motors/hp", MotorAction),
            'ht':actionlib.SimpleActionClient("/motors/ht", MotorAction),
            't':actionlib.SimpleActionClient("/motors/t", MotorAction)
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
            'right':actionlib.SimpleActionClient("/right_hand/move_hand", BHMoveAction),
            'left':actionlib.SimpleActionClient("/left_hand/move_hand", BHMoveAction) }
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
                mcd = subsystem_diag.parseMasterComponentDiag(v.value)
                return mcd
        return None

    def getCoreVeDiag(self):
        """!
        Get diagnostic information for core VE.

        @return Returns object of type subsystem_common.subsystem_diag.SubsystemDiag, with
            diagnostic information about subsystem.
        """
        return self._getSubsystemDiag(self._core_ve_name)

    def getCoreCsDiag(self):
        """!
        Get diagnostic information for core CS.

        @return Returns object of type subsystem_common.subsystem_diag.SubsystemDiag, with
            diagnostic information about subsystem.
        """
        return self._getSubsystemDiag(self._core_cs_name)

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
        self.enableMotor("ht")
        self.enableMotor("t")
        r_hp = self.waitForHP(timeout_s=timeout)
        r_ht = self.waitForHT(timeout_s=timeout)
        r_t = self.waitForT(timeout_s=timeout)
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
        @param imp_list         list of PyKDL.Wrench: End-effector impedance values. This is a path for the impedance of end-effector.
        @param imp_times        list of float: Times of the impedance values in imp_list wrt. start_time.
        @param max_wrench       PyKDL.Wrench: Maximum allowed wrench during the motion.
        @param start_time       float: Relative start time for the movement.
        @param stamp            rospy.Time: Absolute start time for the movement. If both start_time and stamp arguments are set, only stamp is taken into account.
        @param damping          PyKDL.Wrench: Damping for the end-effector.
        @param path_tol         PyKDL.Twist: Maximum allowed error of end-effector pose.

        @return Returns True.

        @exception AssertionError   Raised when prefix is neither 'left' nor 'right'
        """

        assert (prefix=="left" or prefix=="right")

        action_trajectory_goal = CartImpGoal()
        if stamp != None:
            action_trajectory_goal.pose_trj.header.stamp = stamp
        else:
            action_trajectory_goal.pose_trj.header.stamp = rospy.Time.now() + rospy.Duration(start_time)

        if pose_list_T_B_Td:
            i = 0
            for T_B_Td in pose_list_T_B_Td:
                wrist_pose = pm.toMsg(T_B_Td)
                action_trajectory_goal.pose_trj.points.append(CartesianTrajectoryPoint(
                rospy.Duration(pose_times[i]),
                wrist_pose,
                Twist()))
                i += 1

        if tool_list_T_W_T:
            i = 0
            for T_W_T in tool_list_T_W_T:
                tool_pose = pm.toMsg(T_W_T)
                action_trajectory_goal.tool_trj.points.append(CartesianTrajectoryPoint(
                rospy.Duration(tool_times[i]),
                tool_pose,
                Twist()))
                i += 1

        if imp_list:
            i = 0
            for k in imp_list:
                action_trajectory_goal.imp_trj.points.append(CartesianImpedanceTrajectoryPoint(
                rospy.Duration(imp_times[i]),
                CartesianImpedance(self._wrenchKDLtoROS(k), self._wrenchKDLtoROS(damping))))
                i += 1

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

    def waitForEffector(self, prefix, timeout_s=None):
        if timeout_s == None:
            timeout_s = 0
        if not self._action_cart_traj_client[prefix].wait_for_result(timeout=rospy.Duration(timeout_s)):
            return None
        result = self._action_cart_traj_client[prefix].get_result()
        if result.error_code != 0:
            print "waitForEffector(" + prefix + "): action failed with error_code=" + str(result.error_code) + " (" + self._cartesian_trajectory_result_names[result.error_code] + ")"
        return result.error_code

    def waitForEffectorLeft(self, timeout_s=None):
        return self.waitForEffector("left", timeout_s=timeout_s)

    def waitForEffectorRight(self, timeout_s=None):
        return self.waitForEffector("right", timeout_s=timeout_s)

    def maxJointTrajLen(self):
        return 50

    def moveJointTraj(self, traj, joint_names, start_time=0.2, position_tol=5.0/180.0 * math.pi, velocity_tol=5.0/180.0*math.pi):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self._body_joint_names

        pos = traj[0]
        vel = traj[1]
        acc = traj[2]
        dti = traj[3]
        time = 0.0

        js = self.getLastJointState()

        for node_idx in range(0, len(pos)):
            time += dti[node_idx]
            q_dest_all = []
            vel_dest_all = []

            for joint_name in self._body_joint_names:
                if joint_name in joint_names:
                    q_idx = joint_names.index(joint_name)
                    q_dest_all.append(pos[node_idx][q_idx])
                    if vel != None:
                        vel_dest_all.append(vel[node_idx][q_idx])
                    else:
                        vel_dest_all.append(0)
                else:
                    q_dest_all.append(js[1][joint_name])
                    vel_dest_all.append(0)
            goal.trajectory.points.append(JointTrajectoryPoint(q_dest_all, vel_dest_all, [], [], rospy.Duration(time)))

        position_tol = position_tol
        velocity_tol = 5.0/180.0 * math.pi
        acceleration_tol = 5.0/180.0 * math.pi
        for joint_name in goal.trajectory.joint_names:
            goal.path_tolerance.append(JointTolerance(joint_name, position_tol, velocity_tol, acceleration_tol))
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(start_time)
        self._action_joint_traj_client.send_goal(goal)
        return True

    def moveJoint(self, q_dest, joint_names, time, start_time=0.2, position_tol=5.0/180.0 * math.pi, velocity_tol=5.0/180.0*math.pi):
        if joint_names == None:
            if not type(q_dest) is dict:
                print "ERROR: wrong data type passed to moveJoint: ", type(q_dest)
                return False
            joint_names = []
            q_dest_list = []
            for name in q_dest:
                joint_names.append(name)
                q_dest_list.append(q_dest[name])
            return self.moveJointTraj(([q_dest_list], None, None, [time]), joint_names, start_time=start_time, position_tol=position_tol, velocity_tol=velocity_tol)
        else:
            return self.moveJointTraj(([q_dest], None, None, [time]), joint_names, start_time=start_time, position_tol=position_tol, velocity_tol=velocity_tol)

    def waitForJoint(self):
        self._action_joint_traj_client.wait_for_result()
        result = self._action_joint_traj_client.get_result()
        if result.error_code != 0:
            print "waitForJoint(): action failed with error_code=" + str(result.error_code)
        return result.error_code

    def moveHeadTraj(self, traj, start_time=0.2, position_tol=5.0/180.0 * math.pi, velocity_tol=5.0/180.0*math.pi):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["head_pan_joint", "head_tilt_joint"]

        pos = traj[0]
        vel = traj[1]
        acc = traj[2]
        dti = traj[3]
        time = 0.0

        for node_idx in range(0, len(pos)):
            time += dti[node_idx]
            q_dest_all = pos[node_idx]
            if vel != None:
                vel_dest_all = vel[node_idx]
            else:
                vel_dest_all = []
            goal.trajectory.points.append(JointTrajectoryPoint(q_dest_all, vel_dest_all, [], [], rospy.Duration(time)))

        position_tol = position_tol
        velocity_tol = velocity_tol
        acceleration_tol = 5.0/180.0 * math.pi
        for joint_name in goal.trajectory.joint_names:
            goal.path_tolerance.append(JointTolerance(joint_name, position_tol, velocity_tol, acceleration_tol))
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(start_time)
        self._action_head_joint_traj_client.send_goal(goal)
        return True

    def moveHead(self, q_dest, time, start_time=0.2, position_tol=5.0/180.0 * math.pi, velocity_tol=5.0/180.0*math.pi):
        return self.moveHeadTraj(([q_dest], None, None, [time]), start_time=start_time, position_tol=position_tol, velocity_tol=velocity_tol)

    def waitForHead(self):
        self._action_head_joint_traj_client.wait_for_result()
        result = self._action_head_joint_traj_client.get_result()
        if result.error_code != 0:
            print "waitForHead(): action failed with error_code=" + str(result.error_code)
        return result.error_code

#    def checkStopCondition(self, t=0.0):
#
#        end_t = rospy.Time.now()+rospy.Duration(t+0.0001)
#        while rospy.Time.now()<end_t:
#            if rospy.is_shutdown():
#                self.emergencyStop()
#                print "emergency stop: interrupted  %s  %s"%(self.getMaxWrench(), self.wrench_tab_index)
#                self.failure_reason = "user_interrupt"
#                rospy.sleep(1.0)
#            if self.wrench_emergency_stop:
#                self.emergencyStop()
#                print "too big wrench"
#                self.failure_reason = "too_big_wrench"
#                rospy.sleep(1.0)

#            if (self._action_cart_traj_client != None) and (self._action_cart_traj_client.gh) and ((self._action_cart_traj_client.get_state()==GoalStatus.REJECTED) or (self._action_cart_traj_client.get_state()==GoalStatus.ABORTED)):
#                state = self._action_cart_traj_client.get_state()
#                result = self._action_cart_traj_client.get_result()
#                self.emergencyStop()
#                print "emergency stop: traj_err: %s ; %s ; max_wrench: %s   %s"%(state, result, self.getMaxWrench(), self.wrench_tab_index)
#                self.failure_reason = "too_big_wrench_trajectory"
#                rospy.sleep(1.0)

#            if (self._action_tool_client.gh) and ((self._action_tool_client.get_state()==GoalStatus.REJECTED) or (self._action_tool_client.get_state()==GoalStatus.ABORTED)):
#                state = self._action_tool_client.get_state()
#                result = self._action_tool_client.get_result()
#                self.emergencyStop()
#                print "emergency stop: tool_err: %s ; %s ; max_wrench: %s   %s"%(state, result, self.getMaxWrench(), self.wrench_tab_index)
#                self.failure_reason = "too_big_wrench_tool"
#                rospy.sleep(1.0)
#            rospy.sleep(0.01)
#        return self.emergency_stop_active

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
        action_goal.reset = False
        if hold == True:
            action_goal.hold = 1
        else:
            action_goal.hold = 0
        self._action_move_hand_client[prefix].send_goal(action_goal)

    def moveHandLeft(self, q, v, t, maxPressure, hold=False):
        self.moveHand(q, v, t, maxPressure, hold=hold, prefix="left")

    def moveHandRight(self, q, v, t, maxPressure, hold=False):
        self.moveHand(q, v, t, maxPressure, hold=hold, prefix="right")

    def resetHand(self, prefix="right"):
        action_goal = BHMoveGoal()
        action_goal.reset = True
        self._action_move_hand_client[prefix].send_goal(action_goal)

    def resetHandLeft(self):
        self.resetHand(prefix="left")

    def resetHandRight(self):
        self.resetHand(prefix="right")

    def waitForHand(self, prefix="right"):
        self._action_move_hand_client[prefix].wait_for_result()
        result = self._action_move_hand_client[prefix].get_result()
        if result.error_code != 0:
            print "waitForHand(" + prefix + "): action failed with error_code=" + str(result.error_code) + " (" + self._moveHand_action_error_codes_names[result.error_code] + ")"
        return result.error_code

    def waitForHandLeft(self):
        return self.waitForHand(prefix="left")

    def waitForHandRight(self):
        return self.waitForHand(prefix="right")

    def hasContact(self, threshold, print_on_false=False):
        if self.T_F_C != None:
            return True
        return False

    def getKDLtf(self, base_frame, frame):
        pose = self._listener.lookupTransform(base_frame, frame, rospy.Time(0))
        return pm.fromTf(pose)

    def getAllLinksTf(self, base_frame):
        result = {}
        for l in self._all_links:
            try:
                result[l.name] = self.getKDLtf(base_frame, l.name)
            except:
                result[l.name] = None
        return result

    def getTf(self, frame_from, frame_to):
        if frame_from in self._frames and frame_to in self._frames:
            return self.getKDLtf( self._frames[frame_from], self._frames[frame_to] )
        return None

#    def handleEmergencyStop(self):
#        if self.emergency_stop_active:
#            ch = '_'
#            while (ch != 'e') and (ch != 'n') and (ch != 'r'):
#                ch = raw_input("Emergency stop active... (e)xit, (n)ext case, (r)epeat case: ")
#            if ch == 'e':
#                exit(0)
#            if ch == 'n':
#                self.action = "next"
#                self.index += 1
#            if ch == 'r':
#                self.action = "repeat"
#            self.updateTransformations()
#            print "moving desired pose to current pose"
#            self.emergency_stop_active = False
#            self.moveWrist(self.T_B_W, 2.0, Wrench(Vector3(25,25,25), Vector3(5,5,5)))
#            self.checkStopCondition(2.0)
#            return True
#        return False

#    def getMovementTime(self, T_B_Wd, max_v_l = 0.1, max_v_r = 0.2):
#        self.updateTransformations()
#        twist = PyKDL.diff(self.T_B_W, T_B_Wd, 1.0)
#        v_l = twist.vel.Norm()
#        v_r = twist.rot.Norm()
#        f_v_l = v_l/max_v_l
#        f_v_r = v_r/max_v_r
#        if f_v_l > f_v_r:
#            duration = f_v_l
#        else:
#            duration = f_v_r
#        if duration < 0.2:
#            duration = 0.2
#        return duration

#    def getMovementTime2(self, T_B_Wd1, T_B_Wd2, max_v_l = 0.1, max_v_r = 0.2):
#        twist = PyKDL.diff(T_B_Wd1, T_B_Wd2, 1.0)
#        v_l = twist.vel.Norm()
#        v_r = twist.rot.Norm()
#        print "v_l: %s   v_r: %s"%(v_l, v_r)
#        f_v_l = v_l/max_v_l
#        f_v_r = v_r/max_v_r
#        if f_v_l > f_v_r:
#            duration = f_v_l
#        else:
#            duration = f_v_r
#        if duration < 0.5:
#            duration = 0.5
#        return duration

