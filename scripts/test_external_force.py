#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_core_ve_body')

import sys
import rospy
import math
import copy
import tf

from std_msgs.msg import ColorRGBA
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from tf.transformations import * 

if __name__ == "__main__":

    rospy.init_node('cimp_test', anonymous=True)

    rospy.sleep(1)

    rospy.wait_for_service('/gazebo/apply_body_wrench')
    try:
        apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    except rospy.ServiceException, e:
        print "ServiceProxy failed: %s"%e
        exit(0)

    for i in range(3):
        apply_wrench("torso_link0", "world", Point(), Wrench(force=Vector3(), torque=Vector3(0,0,-200)), rospy.Time(), rospy.Duration.from_sec(0.01))
        rospy.sleep(0.5)


