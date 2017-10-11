#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')

import rospy
import math
import PyKDL

from velma_common.velma_interface import *
from control_msgs.msg import FollowJointTrajectoryResult
from planner.planner import *

def exitError(code):
    if code == 0:
        print "OK"
        exit(0)
    print "ERROR:", code
    exit(code)

if __name__ == "__main__":

    rospy.init_node('jimp_test', anonymous=True)

    rospy.sleep(1)

    print "This test/tutorial executes complex motions"\
        " in Joint Impedance mode. Planning is used"\
        " in this example.\n"

    print "Running python interface for Velma..."
    velma = VelmaInterface("/velma_task_cs_ros_interface")
    print "Waiting for VelmaInterface initialization..."
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"

    print "Waiting for Planner initialization..."
    p = Planner(velma.maxJointTrajLen())
    if not p.waitForInit(timeout_s=10.0):
        print "Could not initialize Planner"
        exitError(2)
    print "Planner initialization ok!"

    print "Motors must be enabled every time after the robot enters safe state."
    print "If the motors are already enabled, enabling them has no effect."
    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(2)

    print "Moving to the current position..."
    js_start = velma.getLastJointState()
    velma.moveJoint(js_start[1], None, 1.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(3)

    rospy.sleep(1.0)

    q_map_goal = {'torso_0_joint':0,
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

    print "Moving to valid position, using planned trajectory."
    goal_constraint_1 = qMapToConstraints(q_map_goal, 0.01)
    js = velma.getLastJointState()
    print "Planning..."
    for i in range(3):
        traj, jn = p.plan(js, [goal_constraint_1], "right_arm", max_velocity_scaling_factor=0.2, planner_id="RRTConnect")
        if traj != None:
            break
    if traj == None:
        print "ERROR: planning"
        exitError(4)
    print "Executing trajectory..."
    if not velma.moveJointTraj(traj, jn, start_time=0.5):
        exitError(5)
    if velma.waitForJoint() != 0:
        exitError(6)
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_goal, js):
        exitError(7)

    rospy.sleep(1.0)

    q_map_end = {'torso_0_joint':0,
        'right_arm_0_joint':-0.3,
        'right_arm_1_joint':-1.8,
        'right_arm_2_joint':1.25,
        'right_arm_3_joint':0.85,
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

    q_map_end2 = {'torso_0_joint':0,
        'right_arm_0_joint':-0.3,
        'right_arm_1_joint':-1.8,
        'right_arm_2_joint':-1.25,
        'right_arm_3_joint':-1.57,
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

    print "Moving to starting position, using planned trajectory."
    goal_constraint_2 = qMapToConstraints(q_map_end, 0.01)
    js = velma.getLastJointState()
    print "Planning..."
    for i in range(3):
        traj, jn = p.plan(js, [goal_constraint_2], "right_arm", max_velocity_scaling_factor=0.2, planner_id="RRTConnect")
        if traj != None:
            break
    if traj == None:
        print "ERROR: planning"
        exitError(4)
    print "Executing trajectory..."
    if not velma.moveJointTraj(traj, jn, start_time=0.5):
        exitError(5)
    if velma.waitForJoint() != 0:
        exitError(6)
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_end, js):
        exitError(7)

    exitError(0)

