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

    prefix = "right"

    rospy.sleep(1)


    velma = VelmaInterface("/velma_task_cs_ros_interface")
    print "waiting for init..."

    velma.waitForInit()
    print "init ok"

    js = velma.getLastJointState()
    joint_names = []
    q_dest = []
    for joint_name in js[1]:
        joint_names.append(joint_name)
        q_dest.append(js[1][joint_name])

    velma.moveJoint(q_dest, joint_names, 5.0, start_time=0.5)
    velma.waitForJoint()

