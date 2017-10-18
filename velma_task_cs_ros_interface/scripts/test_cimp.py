#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy

from velma_common.velma_interface import *
from planner.planner import *

def exitError(code):
    if code == 0:
        print "OK"
        exit(0)
    print "ERROR:", code
    exit(code)

if __name__ == "__main__":

    rospy.init_node('test_cimp', anonymous=False)

    rospy.sleep(1)

    velma = VelmaInterface("/velma_task_cs_ros_interface")
    print "waiting for VelmaInterface init..."
    if not velma.waitForInit(timeout_s=20):
        print "could not initialize VelmaInterface"
        exitError(1)
    print "VelmaInterface init ok"

    print "waiting for Planner init..."
    p = Planner(velma.maxJointTrajLen())
    if not p.waitForInit():
        print "could not initialize PLanner"
        exitError(2)
    print "Planner init ok"

    if velma.enableMotors() != 0:
        exitError(14)

    print "sending head pan START_HOMING command"
    velma.startHomingHP()
    if velma.waitForHP() != 0:
        exitError(14)

    print "sending head tilt START_HOMING command"
    velma.startHomingHT()
    if velma.waitForHT() != 0:
        exitError(15)

    q_map_1 = {'torso_0_joint':0.0,
        'right_arm_0_joint':-0.3,
        'right_arm_1_joint':-1.57,
        'right_arm_2_joint':1.57,
        'right_arm_3_joint':1.57,
        'right_arm_4_joint':0.0,
        'right_arm_5_joint':-1.57,
        'right_arm_6_joint':0.0,
        'left_arm_0_joint':0.3,
        'left_arm_1_joint':1.57,
        'left_arm_2_joint':-1.57,
        'left_arm_3_joint':-1.7,
        'left_arm_4_joint':0.0,
        'left_arm_5_joint':1.57,
        'left_arm_6_joint':0.0
        }

    goal_constraint_1 = qMapToConstraints(q_map_1, 0.01)

    print "moving whole body to initial pose (jimp)"
    js = velma.getLastJointState()
    for i in range(10):
        traj, jn = p.plan(js, [goal_constraint_1], "impedance_joints", max_velocity_scaling_factor=0.1)
        if traj != None:
            break
    if traj == None:
        print "ERROR: planning"
        exitError(4)
    if not velma.moveJointTraj(traj, jn, start_time=0.5):
        exitError(5)
    if velma.waitForJoint() != 0:
        exitError(6)
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_1, js[1]):
        exitError(7)

    print "moving right arm to another pose (cimp)"
    T_B_Trd = velma.getTf("B", "Wr")
    T_B_Trd_init = copy.copy(T_B_Trd)
    T_B_Tld_init = copy.copy(velma.getTf("B", "Wl"))
    T_B_Trd = PyKDL.Frame(PyKDL.Vector(0,0,0.1)) * T_B_Trd
    if not velma.moveEffectorRight(T_B_Trd, 3.0, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, stamp=None, path_tol=None):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)

    rospy.sleep(0.5)

    print "The rest of this test is not ready yet"
    exitError(0)

    print "moving right arm to another pose (cimp, error - too fast)"
    T_B_Trd = velma.getTf("B", "Wr")
    T_B_Trd = PyKDL.Frame(PyKDL.Vector(0,0,1.1)) * T_B_Trd
    if not velma.moveEffectorRight(T_B_Trd, 0.1, PyKDL.Wrench(PyKDL.Vector(50,50,50), PyKDL.Vector(50,50,50)), start_time=0.5, stamp=None, path_tol=None):
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
    traj, jn = p.plan(js, [goal_constraint_1], "impedance_joints", max_velocity_scaling_factor=0.1)
    if not velma.moveJointTraj(traj, jn, start_time=0.5):
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
    if not velma.moveEffectorRight(T_B_Trd, 5.0, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, stamp=None, path_tol=PyKDL.Twist(PyKDL.Vector(0.02, 0.02, 0.02), PyKDL.Vector(0.05, 0.05, 0.05))):
        exitError(15)
    if not velma.moveEffectorLeft(T_B_Tld, 5.0, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, stamp=None, path_tol=PyKDL.Twist(PyKDL.Vector(0.02, 0.02, 0.02), PyKDL.Vector(0.05, 0.05, 0.05))):
        exitError(16)
    if velma.waitForEffectorRight() != -5:
        exitError(17)
    if velma.waitForEffectorLeft() != -5:
        exitError(18)

    print "waiting 2 seconds..."
    rospy.sleep(2)

    print "moving arms to initial pose (cimp)"
    if not velma.moveEffectorRight(T_B_Trd_init, 5.0, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, stamp=None, path_tol=None):
        exitError(19)
    if not velma.moveEffectorLeft(T_B_Tld_init, 5.0, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, stamp=None, path_tol=None):
        exitError(20)
    if velma.waitForEffectorRight() != 0:
        exitError(21)
    if velma.waitForEffectorLeft() != 0:
        exitError(22)

    exitError(0)

