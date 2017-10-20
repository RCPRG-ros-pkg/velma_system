#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')

import rospy
import math
import PyKDL

from velma_common.velma_interface import *
from control_msgs.msg import FollowJointTrajectoryResult

def exitError(code):
    if code == 0:
        print "OK"
        exit(0)
    print "ERROR:", code
    exit(code)

if __name__ == "__main__":

    rospy.init_node('jimp_test', anonymous=True)

    rospy.sleep(0.5)

    print "This test/tutorial executes simple motions"\
        " in Joint Impedance mode. Planning is not used"\
        " in this example.\n"

    print "Running python interface for Velma..."
    velma = VelmaInterface()
    print "Waiting for VelmaInterface initialization..."
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"

    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(1)

    print "Motors must be enabled every time after the robot enters safe state."
    print "If the motors are already enabled, enabling them has no effect."
    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(2)

    print "Moving to the current position..."
    velma.moveJointImpToCurrentPos(start_time=0.5)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(3)

    q_map_0 = {'torso_0_joint':0,
        'right_arm_0_joint':0,
        'right_arm_1_joint':0,
        'right_arm_2_joint':0,
        'right_arm_3_joint':0,
        'right_arm_4_joint':0,
        'right_arm_5_joint':0,
        'right_arm_6_joint':0,
        'left_arm_0_joint':0,
        'left_arm_1_joint':0,
        'left_arm_2_joint':0,
        'left_arm_3_joint':0,
        'left_arm_4_joint':0,
        'left_arm_5_joint':0,
        'left_arm_6_joint':0
        }

    q_map_starting = {'torso_0_joint':0,
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

    print "Moving to position 0 (this motion is too fast and should cause error condition, that leads to safe mode in velma_core_cs)."
    velma.moveJoint(q_map_0, 0.1, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
        print "The action should have ended with PATH_TOLERANCE_VIOLATED error status, but the error code is", error
        exitError(4)

    print "waiting 2 seconds..."
    rospy.sleep(2)

    print "Motors must be enabled every time after the robot enters safe state."
    print "If the motors are already enabled, enabling them has no effect."
    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(5)

    print "Moving to position 0 (slowly)."
    velma.moveJoint(q_map_0, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()

    print "Moving to the starting position..."
    velma.moveJoint(q_map_starting, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(6)

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

    print "Moving to valid position, using invalid self-colliding trajectory (this motion should cause error condition, that leads to safe mode in velma_core_cs)."
    velma.moveJoint(q_map_goal, 9.0, start_time=0.5, position_tol=5.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
        print "The action should have ended with PATH_TOLERANCE_VIOLATED error status, but the error code is", error
        exitError(7)

    print "waiting 2 seconds..."
    rospy.sleep(2)

    print "Motors must be enabled every time after the robot enters safe state."
    print "If the motors are already enabled, enabling them has no effect."
    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(8)

    print "Moving to the starting position..."
    velma.moveJoint(q_map_starting, 4.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(9)

    q_map_intermediate = {'torso_0_joint':0,
        'right_arm_0_joint':-0.3,
        'right_arm_1_joint':-1.6,
        'right_arm_2_joint':-1.25,
        'right_arm_3_joint':-0.85,
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

    print "To reach the goal position, some trajectory must be exetuted that contains additional, intermediate nodes"

    print "Using SafeCol behavior to exit self collision..."
    velma.switchToSafeColBehavior()

    rospy.sleep(2)

    print "Moving to the intermediate position..."
    velma.moveJoint(q_map_intermediate, 8.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(10)

    print "Moving to the goal position."
    velma.moveJoint(q_map_goal, 8.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended with PATH_TOLERANCE_VIOLATED error status, but the error code is", error
        exitError(11)

    print "Moving to the intermediate position..."
    velma.moveJoint(q_map_intermediate, 8.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(12)

    print "Moving to the starting position..."
    velma.moveJoint(q_map_starting, 5.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(13)

    exitError(0)

