#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy

from velma_common.velma_interface import *
from planner.planner import *

from moveit_msgs.msg import *
from moveit_msgs.srv import *

if __name__ == "__main__":

    print "This test is not ready yet"
    exit(0)

    rospy.init_node('wrists_test', anonymous=True)

    rospy.sleep(1)

    p = Planner()

    velma = VelmaInterface("/velma_task_cs_ros_interface")
    print "waiting for init..."
    velma.waitForInit()
    print "init ok"

    q_map_1 = {'torso_0_joint':0.0,
        'right_arm_0_joint':0,#-0.3,
        'right_arm_1_joint':0,#-1.57,
        'right_arm_2_joint':0,#1.57,
        'right_arm_3_joint':0,#1.57,
        'right_arm_4_joint':0.0,
        'right_arm_5_joint':-1.57,
        'right_arm_6_joint':0.0,
        'left_arm_0_joint':0,#0.3,
        'left_arm_1_joint':0,#1.57,
        'left_arm_2_joint':0,#-1.57,
        'left_arm_3_joint':0,#-1.7,
        'left_arm_4_joint':0.0,
        'left_arm_5_joint':1.57,
        'left_arm_6_joint':0.0
        }

    q_map_2 = {'torso_0_joint':0.0,
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

    goal_constraint_1 = qMapToConstraints(q_map_2, 0.01)

    js = velma.getLastJointState()
    traj = p.plan(js[1], [goal_constraint_1], "impedance_joints", max_velocity_scaling_factor=0.1)

    velma.moveJointTraj(traj, start_time=0.5)
    velma.waitForJoint()

    rospy.sleep(0.5)

    print "moving right arm to another pose (cimp)"
    T_B_Trd = velma.getTf("B", "Wr") * PyKDL.Frame(PyKDL.Rotation.RotY(160))
    T_B_Tld = velma.getTf("B", "Wl") * PyKDL.Frame(PyKDL.Rotation.RotY(-160))
#    T_B_Trd = velma.getTf("B", "Wr") * PyKDL.Frame(PyKDL.Rotation.RotZ(-160))
#    T_B_Tld = velma.getTf("B", "Wl") * PyKDL.Frame(PyKDL.Rotation.RotZ(160))
    velma.moveEffectorRight(T_B_Trd, 5.0, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, stamp=None, path_tol=None)
    velma.moveEffectorLeft(T_B_Tld, 5.0, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, stamp=None, path_tol=None)
    velma.waitForEffectorRight()
    velma.waitForEffectorLeft()

    rospy.sleep(2.0)

    goal_constraint_2 = qMapToConstraints(q_map_2, 0.01)

    js = velma.getLastJointState()
    traj = p.plan(js[1], [goal_constraint_2], "impedance_joints", max_velocity_scaling_factor=0.1)

    velma.moveJointTraj(traj, start_time=0.5)
    velma.waitForJoint()

