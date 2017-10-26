#!/usr/bin/env python

## Runs test for motions in cart_imp mode.
# @ingroup integration_tests
# @file test_cimp_pose.py
# @namespace scripts.test_cimp_pose Integration test

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy

from velma_common import *
from planner import *
from rcprg_ros_utils import exitError

if __name__ == "__main__":
    # define some configurations
    q_map_starting = {'torso_0_joint':0,
        'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0,      'left_arm_4_joint':0,
        'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

    q_map_1 = {'torso_0_joint':0.0,
        'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.57,  'left_arm_1_joint':1.57,
        'right_arm_2_joint':1.57,   'left_arm_2_joint':-1.57,
        'right_arm_3_joint':1.57,   'left_arm_3_joint':-1.7,
        'right_arm_4_joint':0.0,    'left_arm_4_joint':0.0,
        'right_arm_5_joint':-1.57,  'left_arm_5_joint':1.57,
        'right_arm_6_joint':0.0,    'left_arm_6_joint':0.0 }

    rospy.init_node('test_cimp_pose', anonymous=False)

    rospy.sleep(0.5)

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

    print "waiting for Planner init..."
    p = Planner(velma.maxJointTrajLen())
    if not p.waitForInit():
        print "could not initialize PLanner"
        exitError(2)
    print "Planner init ok"

    if velma.enableMotors() != 0:
        exitError(14)

    print "Moving to the current position (jnt_imp)..."
    velma.moveJointImpToCurrentPos(start_time=0.2)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(3)

    rospy.sleep(0.5)

    diag = velma.getCoreCsDiag()
    if not diag.inStateJntImp():
        print "The core_cs should be in jnt_imp state, but it is not"
        exitError(3)

    print "Checking if the starting configuration is as expected..."
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_starting, js[1], tolerance=0.2):
        print "This test requires starting pose:"
        print q_map_starting
        exitError(10)

    print "Moving to the current position (cart_imp)..."
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)

    rospy.sleep(0.5)

    diag = velma.getCoreCsDiag()
    if not diag.inStateCartImp():
        print "The core_cs should be in jnt_imp state, but it is not"
        exitError(3)

    # get initial configuration
    js_init = velma.getLastJointState()

    print "Planning motion to the goal position using set of all joints..."

    print "Moving to valid position, using planned trajectory."
    goal_constraint_1 = qMapToConstraints(q_map_1, 0.01, group=velma.getJointGroup("impedance_joints"))
    for i in range(5):
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        print "Planning (try", i, ")..."
        traj = p.plan(js[1], [goal_constraint_1], "impedance_joints", max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
        if traj == None:
            continue
        print "Executing trajectory..."
        if not velma.moveJointTraj(traj, start_time=0.5):
            exitError(5)
        if velma.waitForJoint() == 0:
            break
        else:
            print "The trajectory could not be completed, retrying..."
            continue

    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_1, js[1]):
        exitError(6)

    print "moving right arm to another pose (cimp)"
    T_B_Trd = velma.getTf("B", "Wr")
    T_B_Trd_init = copy.copy(T_B_Trd)
    T_B_Tld_init = copy.copy(velma.getTf("B", "Wl"))
    T_B_Trd = PyKDL.Frame(PyKDL.Vector(0,0,0.1)) * T_B_Trd

    if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)

    rospy.sleep(0.5)

    print "Planning motion to the starting position using set of all joints..."

    print "Moving to valid position, using planned trajectory."
    goal_constraint_2 = qMapToConstraints(q_map_starting, 0.01, group=velma.getJointGroup("impedance_joints"))
    for i in range(5):
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        print "Planning (try", i, ")..."
        traj = p.plan(js[1], [goal_constraint_2], "impedance_joints", max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
        if traj == None:
            continue
        print "Executing trajectory..."
        if not velma.moveJointTraj(traj, start_time=0.5):
            exitError(5)
        if velma.waitForJoint() == 0:
            break
        else:
            print "The trajectory could not be completed, retrying..."
            continue

    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_starting, js[1]):
        exitError(6)

    print "The rest of this test is not ready yet"
    exitError(0)

    #TODO: implement more motions

    print "moving right arm to another pose (cimp, error - too fast)"
    T_B_Trd = velma.getTf("B", "Wr")
    T_B_Trd = PyKDL.Frame(PyKDL.Vector(0,0,1.1)) * T_B_Trd
    if not velma.moveCartImpRight([T_B_Trd], [0.1], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(50,50,50), PyKDL.Vector(50,50,50)), start_time=0.5):
        exitError(10)
    if velma.waitForEffectorRight() != -5:
        exitError(11)

    rospy.sleep(2.0)

    if velma.enableMotors() != 0:
        exitError(14)

    print "waiting 2 seconds..."
    rospy.sleep(2)

    print "moving whole body to initial pose (jimp)"
    js = velma.getLastJointState()
    traj = p.plan(js[1], [goal_constraint_1], "impedance_joints", max_velocity_scaling_factor=0.1)
    if not velma.moveJointTraj(traj, start_time=0.5):
        exitError(12)
    if velma.waitForJoint() != 0:
        exitError(13)
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_1, js[1]):
        exitError(14)

    print "moving arms to self-collision pose (cimp)"
    T_B_Trd = PyKDL.Frame(PyKDL.Vector(0.4, 0.1, 1.2))
    T_B_Tld = PyKDL.Frame(PyKDL.Rotation.RotZ(180.0/180.0*math.pi), PyKDL.Vector(0.4, -0.1, 1.2))
    if not velma.moveCartImpRight([T_B_Trd], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, path_tol=PyKDL.Twist(PyKDL.Vector(0.02, 0.02, 0.02), PyKDL.Vector(0.05, 0.05, 0.05))):
        exitError(15)
    if not velma.moveCartImpLeft([T_B_Tld], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, path_tol=PyKDL.Twist(PyKDL.Vector(0.02, 0.02, 0.02), PyKDL.Vector(0.05, 0.05, 0.05))):
        exitError(16)
    if velma.waitForEffectorRight() != -5:
        exitError(17)
    if velma.waitForEffectorLeft() != -5:
        exitError(18)

    print "waiting 2 seconds..."
    rospy.sleep(2)

    print "moving arms to initial pose (cimp)"
    if not velma.moveCartImpRight([T_B_Trd_init], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(19)
    if not velma.moveCartImpLeft([T_B_Tld_init], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(20)
    if velma.waitForEffectorRight() != 0:
        exitError(21)
    if velma.waitForEffectorLeft() != 0:
        exitError(22)

    exitError(0)

