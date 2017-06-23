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

    print "move left"
    velma.moveHandLeft([90.0/180.0*math.pi,0,0,0], [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    velma.waitForHandLeft()

    print "move right"
    velma.moveHandRight([90.0/180.0*math.pi,0,0,0], [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    velma.waitForHandRight()

    print "move right"
    velma.moveHandRight([90.0/180.0*math.pi,0,0,180.0/180.0*math.pi], [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    velma.waitForHandRight()

    print "reset left"
    velma.resetHandLeft()
    velma.waitForHandLeft()

    print "reset right"
    velma.resetHandRight()
    velma.waitForHandRight()

#    print "move left"
#    velma.moveHandLeft([0,0,0,0], [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
#    velma.waitForHandLeft()

#    print "move right"
#    velma.moveHandRight([0,0,0,0], [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
#    velma.waitForHandRight()

