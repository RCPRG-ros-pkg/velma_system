#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy

from velma_common.velma_interface import *
from planner.planner import *

from moveit_msgs.msg import *
from moveit_msgs.srv import *

if __name__ == "__main__":

    rospy.init_node('wrists_test', anonymous=True)

    rospy.sleep(1)

    p = Planner()

    velma = VelmaInterface("/velma_task_cs_ros_interface")
    print "waiting for init..."
    velma.waitForInit()
    print "init ok"

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

    goal_constraint_1 = Constraints()
    for joint_name in q_map_1:
        constraint = JointConstraint()
        constraint.joint_name = joint_name
        constraint.position = q_map_1[ joint_name ]
        constraint.tolerance_above = 0.1
        constraint.tolerance_below = 0.1
        constraint.weight = 1.0
        goal_constraint_1.joint_constraints.append( constraint )

    js = velma.getLastJointState()
    traj, jn = p.plan(js, [goal_constraint_1], "impedance_joints", max_velocity_scaling_factor=0.1)

    velma.moveJointTraj(traj, jn, start_time=0.5)
    velma.waitForJoint()

    q_map_2 = {'torso_0_joint':0.0,
        'right_arm_0_joint':-0.3,
        'right_arm_1_joint':-1.57,
        'right_arm_2_joint':1.57,
        'right_arm_3_joint':1.57,
        'right_arm_4_joint':0.0,
        'right_arm_5_joint':1.57,
        'right_arm_6_joint':2.8,
        'left_arm_0_joint':0.3,
        'left_arm_1_joint':1.57,
        'left_arm_2_joint':-1.57,
        'left_arm_3_joint':-1.7,
        'left_arm_4_joint':0.0,
        'left_arm_5_joint':-1.57,
        'left_arm_6_joint':2.8
        }

    goal_constraint_2 = Constraints()
    for joint_name in q_map_2:
        constraint = JointConstraint()
        constraint.joint_name = joint_name
        constraint.position = q_map_2[ joint_name ]
        constraint.tolerance_above = 0.1
        constraint.tolerance_below = 0.1
        constraint.weight = 1.0
        goal_constraint_2.joint_constraints.append( constraint )

    rospy.sleep(0.5)
    js = velma.getLastJointState()
    traj, jn = p.plan(js, [goal_constraint_2], "impedance_joints", max_velocity_scaling_factor=0.1)
    velma.moveJointTraj(traj, jn, start_time=0.5)
    velma.waitForJoint()

    rospy.sleep(0.5)
    js = velma.getLastJointState()
    traj, jn = p.plan(js, [goal_constraint_1], "impedance_joints", max_velocity_scaling_factor=0.1)
    velma.moveJointTraj(traj, jn, start_time=0.5)
    velma.waitForJoint()

