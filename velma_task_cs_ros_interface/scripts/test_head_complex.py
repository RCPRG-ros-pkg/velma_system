#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')

import rospy
import copy

from velma_common.velma_interface import *

def exitError(code):
    if code == 0:
        print "OK"
        exit(0)
    print "ERROR:", code
    exit(code)

if __name__ == "__main__":

    rospy.init_node('head_test', anonymous=True)

    rospy.sleep(0.5)

    print "This test/tutorial executes complex motions"\
        " of head along with motions in Joint Impedance mode."

    print "Running python interface for Velma..."
    velma = VelmaInterface()
    print "Waiting for VelmaInterface initialization..."
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"

    print "Motors must be enabled every time after the robot enters safe state."
    print "If the motors are already enabled, enabling them has no effect."
    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(2)

    print "Moving to the current position..."
    js_start = velma.getLastJointState()
    velma.moveJoint(js_start[1], 0.5, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(3)

    print "moving head to position: left"
    q_dest = (1.56, 0)
    velma.moveHead(q_dest, 2.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(4)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(5)

    print "moving head to position: right (interrupted by invalid motion in Joint Impedance mode)"
    q_dest = (-1.56, 0)
    velma.moveHead(q_dest, 10.0, start_time=0.5)

    rospy.sleep(0.5)

    print "Moving too fast to another position (safe mode in velma_core_ve_body)..."
    q_map = copy.copy(js_start[1])
    q_map['torso_0_joint'] = 1.0
    velma.moveJoint(q_map, 0.05, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error == 0:
        print "The action should have ended with error, but the error code is", error
        exitError(6)

    if velma.waitForHead() == 0:
        exitError(7)
    rospy.sleep(0.5)

    if isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(8)

    rospy.sleep(1.0)

    print "Motors must be enabled every time after the robot enters safe state."
    print "If the motors are already enabled, enabling them has no effect."
    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(9)

    print "Moving to the current position..."
    js = velma.getLastJointState()
    velma.moveJoint(js[1], 0.5, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(10)

    print "Checking if old trajectory for head is continued..."
    rospy.sleep(2.0)
    if isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(11)

    print "moving head to position: left"
    q_dest = (1.56, 0)
    velma.moveHead(q_dest, 2.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(12)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(13)

    print "moving head to position: right (interrupted by invalid motion in Joint Impedance mode)"
    q_dest = (-1.56, 0)
    velma.moveHead(q_dest, 10.0, start_time=0.5)

    rospy.sleep(0.5)

    print "Moving to self-collision pose (safe mode in velma_core_cs)..."
    q_map = {'torso_0_joint':0,
        'right_arm_0_joint':-0.3,
        'right_arm_1_joint':-1.8,
        'right_arm_2_joint':-1.25,
        'right_arm_3_joint':1.57,
        'right_arm_4_joint':0,
        'right_arm_5_joint':-0.5,
        'right_arm_6_joint':0,
        'left_arm_0_joint':0.3,
        'left_arm_1_joint':1.8,
        'left_arm_2_joint':-1.25,
        'left_arm_3_joint':-0.85,
        'left_arm_4_joint':0,
        'left_arm_5_joint':0.5,
        'left_arm_6_joint':0
        }
    velma.moveJoint(q_map, 4.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error == 0:
        print "The action should have ended with error, but the error code is", error
        exitError(14)

    if velma.waitForHead() == 0:
        exitError(15)
    rospy.sleep(0.5)

    if isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(16)

    rospy.sleep(1.0)

    print "Motors must be enabled every time after the robot enters safe state."
    print "If the motors are already enabled, enabling them has no effect."
    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(17)

    print "Moving to the current position..."
    js = velma.getLastJointState()
    velma.moveJoint(js[1], 0.5, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(18)

    print "Checking if old trajectory for head is continued..."
    rospy.sleep(2.0)
    if isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(19)

    print "moving head to position: 0"
    q_dest = (0,0)
    velma.moveHead(q_dest, 2.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(20)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(21)

    exitError(0)

