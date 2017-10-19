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

    print "moving to initial position..."
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
    js = velma.getLastJointState()
    traj = p.plan(js[1], [goal_constraint_1], "impedance_joints", max_velocity_scaling_factor=0.1)
    velma.moveJointTraj(traj, start_time=0.5)
    velma.waitForJoint()

    rospy.sleep(0.5)

    print "moving to 'inverted wrist' position..."
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

    goal_constraint_2 = qMapToConstraints(q_map_2, 0.01)
    js = velma.getLastJointState()
    traj = p.plan(js[1], [goal_constraint_2], "impedance_joints", max_velocity_scaling_factor=0.1)
    velma.moveJointTraj(traj, start_time=0.5)
    velma.waitForJoint()

    rospy.sleep(0.5)

    print "moving to 'tool inertia test' position..."
    q_map_3 = {'torso_0_joint':0.0,
        'right_arm_0_joint':-0.3,
        'right_arm_1_joint':-1.57,
        'right_arm_2_joint':1.57,
        'right_arm_3_joint':1.57,
        'right_arm_4_joint':0.0,
        'right_arm_5_joint':-1.57,
        'right_arm_6_joint':1.57,
        'left_arm_0_joint':0.3,
        'left_arm_1_joint':1.57,
        'left_arm_2_joint':-1.57,
        'left_arm_3_joint':-1.7,
        'left_arm_4_joint':0.0,
        'left_arm_5_joint':1.57,
        'left_arm_6_joint':-1.57
        }

    goal_constraint_3 = qMapToConstraints(q_map_3, 0.01)
    js = velma.getLastJointState()
    traj = p.plan(js[1], [goal_constraint_3], "impedance_joints", max_velocity_scaling_factor=0.1)
    velma.moveJointTraj(traj, start_time=0.5)
    velma.waitForJoint()


    print "changing grippers configuration..."
    velma.moveHandLeft([0,0,0,0], [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    velma.moveHandRight([0,0,0,0], [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    velma.waitForHandLeft()
    velma.waitForHandRight()

    velma.moveHandLeft([120.0/180.0*math.pi,120.0/180.0*math.pi,120.0/180.0*math.pi,0], [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    velma.moveHandRight([120.0/180.0*math.pi,120.0/180.0*math.pi,120.0/180.0*math.pi,0], [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    velma.waitForHandLeft()
    velma.waitForHandRight()

    velma.moveHandLeft([0,0,0,0], [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    velma.moveHandRight([0,0,0,0], [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    velma.waitForHandLeft()
    velma.waitForHandRight()

    velma.moveHandLeft([0,0,0,180.0/180.0*math.pi], [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    velma.moveHandRight([0,0,0,180.0/180.0*math.pi], [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    velma.waitForHandLeft()
    velma.waitForHandRight()

    velma.moveHandLeft([0,0,0,0], [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    velma.moveHandRight([0,0,0,0], [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    velma.waitForHandLeft()
    velma.waitForHandRight()

    rospy.sleep(0.5)

    print "moving to 'wrist singularity' position..."
    q_map_4 = {'torso_0_joint':0.0,
        'right_arm_0_joint':-0.3,
        'right_arm_1_joint':-1.57,
        'right_arm_2_joint':1.57,
        'right_arm_3_joint':1.57,
        'right_arm_4_joint':0.0,
        'right_arm_5_joint':0,
        'right_arm_6_joint':0,
        'left_arm_0_joint':0.3,
        'left_arm_1_joint':1.57,
        'left_arm_2_joint':-1.57,
        'left_arm_3_joint':-1.7,
        'left_arm_4_joint':0.0,
        'left_arm_5_joint':0,
        'left_arm_6_joint':0
        }

    goal_constraint_4 = qMapToConstraints(q_map_4, 0.01)
    js = velma.getLastJointState()
    traj = p.plan(js[1], [goal_constraint_4], "impedance_joints", max_velocity_scaling_factor=0.1)
    velma.moveJointTraj(traj, start_time=0.5)
    velma.waitForJoint()

    rospy.sleep(0.5)

    print "moving to initial position..."
    js = velma.getLastJointState()
    traj = p.plan(js[1], [goal_constraint_1], "impedance_joints", max_velocity_scaling_factor=0.1)
    velma.moveJointTraj(traj, start_time=0.5)
    velma.waitForJoint()

