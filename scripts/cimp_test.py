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

    prefix = "right"

    rospy.sleep(1)


    velma = VelmaInterface("/velma_task_cs_ros_interface")
    print "waiting for init..."

    velma.waitForInit()
    print "init ok"

    exit(0)

    listener = tf.TransformListener();

    action_trajectory_client = actionlib.SimpleActionClient("/" + prefix + "_arm/cartesian_trajectory", CartesianTrajectoryAction)
    print "cimp_test script: waiting for /%s_arm/cartesian_trajectory action..."%(prefix)
    action_trajectory_client.wait_for_server()

    print "connected to the action server"

    T_B_Td = PyKDL.Frame()
    pose = pm.toMsg(T_B_Td)
    duration = 20.0

    action_trajectory_goal = CartesianTrajectoryGoal()
            
    action_trajectory_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
            
    action_trajectory_goal.trajectory.points.append(CartesianTrajectoryPoint(
    rospy.Duration(duration),
    pose,
    Twist()))

    action_trajectory_client.send_goal(action_trajectory_goal)

    rospy.spin()

