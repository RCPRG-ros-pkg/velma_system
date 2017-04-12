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

    print "This script tests Vitrual Effector of Velma robot for"
    print "This script tests Vitrual Effector of Velma robot for"
    rospy.sleep(1)

    try:
        rospy.wait_for_service('/gazebo/apply_body_wrench', timeout=10.0)
        rospy.wait_for_service('/gazebo/get_model_properties', timeout=10.0)
        rospy.wait_for_service('/gazebo/get_joint_properties', timeout=10.0)
    except rospy.ServiceException, e:
        print "wait_for_service failed: %s"%e
        exit(0)

    try:
        apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    except rospy.ServiceException, e:
        print "ServiceProxy failed: %s"%e
        exit(0)

    try:
        get_model_properties = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
    except rospy.ServiceException, e:
        print "ServiceProxy failed: %s"%e
        exit(0)

    try:
        get_joint_proporties = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
    except rospy.ServiceException, e:
        print "ServiceProxy failed: %s"%e
        exit(0)

    model_prop = get_model_properties("velma")
    if not model_prop.success:
        print "error while executing rosservice '/gazebo/get_model_properties':", model_prop.status_message
        exit(0)

    print "applying external forces..."
    for i in range(3):
        apply_wrench("torso_link0", "world", Point(), Wrench(force=Vector3(), torque=Vector3(0,0,200)), rospy.Time(), rospy.Duration.from_sec(0.01))
        apply_wrench("left_arm_4_link", "world", Point(), Wrench(force=Vector3(), torque=Vector3(0,0,50)), rospy.Time(), rospy.Duration.from_sec(0.01))
        apply_wrench("right_arm_4_link", "world", Point(), Wrench(force=Vector3(), torque=Vector3(0,0,50)), rospy.Time(), rospy.Duration.from_sec(0.01))
        rospy.sleep(0.5)

    print "waitning for stabilization..."
    rospy.sleep(2.0)

    joint_pos = {}
    for joint_name in model_prop.joint_names:
        joint_prop = get_joint_proporties(joint_name)
        if not joint_prop.success:
            print "error while executing rosservice '/gazebo/get_joint_properties' for joint", joint_name, ":", model_prop.status_message
            exit(0)
        joint_pos[joint_name] = joint_prop.position[0]

    print "waitning for next measurement..."
    rospy.sleep(1.0)
    for joint_name in model_prop.joint_names:
        joint_prop = get_joint_proporties(joint_name)
        if not joint_prop.success:
            print "error while executing rosservice '/gazebo/get_joint_properties' for joint", joint_name, ":", model_prop.status_message
            exit(0)
        diff = joint_prop.position[0] - joint_pos[joint_name]
        if abs(diff) > 1.0/180.0*math.pi:
            print "joint", joint_name, "is still moving:", diff
#        else:
#            print "joint", joint_name, "is stable:", diff

