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

    rospy.init_node('head_test', anonymous=True)

    rospy.sleep(1)

    velma = VelmaInterface("/velma_task_cs_ros_interface")
    print "waiting for init..."

    velma.waitForInit()
    print "init ok"


    print "moving head to position: 0"
    q_dest = (0,0)
    velma.moveHead(q_dest, 2.0, start_time=0.5)
    velma.waitForHead()

    print "moving head to position: left"
    q_dest = (1.5, 0)
    velma.moveHead(q_dest, 2.0, start_time=0.5)
    velma.waitForHead()

    print "moving head to position: 0"
    q_dest = (0,0)
    velma.moveHead(q_dest, 2.0, start_time=0.5)
    velma.waitForHead()

    print "moving head to position: right"
    q_dest = (-1.5, 0)
    velma.moveHead(q_dest, 4.0, start_time=0.5)
    velma.waitForHead()

    print "moving head to position: up"
    q_dest = (0, -0.9)
    velma.moveHead(q_dest, 4.0, start_time=0.5)
    velma.waitForHead()

    print "moving head to position: 0"
    q_dest = (0,0)
    velma.moveHead(q_dest, 2.0, start_time=0.5)
    velma.waitForHead()

    print "moving head to position: down"
    q_dest = (0, 0.9)
    velma.moveHead(q_dest, 4.0, start_time=0.5)
    velma.waitForHead()

    print "moving head to position: 0"
    q_dest = (0,0)
    velma.moveHead(q_dest, 2.0, start_time=0.5)
    velma.waitForHead()

