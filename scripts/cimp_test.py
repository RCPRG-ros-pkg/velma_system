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

    print "moving right arm to current pose"
    T_B_Trd = velma.getTf("B", "Wr")
    velma.moveEffectorRight(T_B_Trd, 2.0, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, stamp=None, path_tol=None)
    velma.waitForEffectorRight()


    print "moving right arm to another pose"
    T_B_Trd = velma.getTf("B", "Wr")
    T_B_Trd = T_B_Trd * PyKDL.Frame(PyKDL.Vector(0.1,0,0))
    velma.moveEffectorRight(T_B_Trd, 10.0, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, stamp=None, path_tol=None)
    velma.waitForEffectorRight()

