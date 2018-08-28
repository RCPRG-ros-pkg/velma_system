#!/usr/bin/env python

## Runs test for jnt_imp mode motion.
# @ingroup integration_tests
# @file test_jimp.py
# @namespace scripts.test_jimp Integration test

# Copyright (c) 2017, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Dawid Seredynski
#

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')

import rospy
import math
import PyKDL

from velma_common.velma_interface import *
from trapezoid_trajectory_msgs.msg import TrapezoidTrajectoryResult


def showConfig():
    js = velma.getLastJointState()
    print 'torso_0_joint: ',round(js[1]['torso_0_joint'],2),'\n',\
          'right_arm_0_joint: ',round(js[1]['right_arm_0_joint'],2),'\n',\
          'right_arm_1_joint: ',round(js[1]['right_arm_1_joint'],2),'\n',\
          'right_arm_2_joint: ',round(js[1]['right_arm_2_joint'],2),'\n',\
          'right_arm_3_joint: ',round(js[1]['right_arm_3_joint'],2),'\n',\
          'right_arm_4_joint: ',round(js[1]['right_arm_4_joint'],2),'\n',\
          'right_arm_5_joint: ',round(js[1]['right_arm_5_joint'],2),'\n',\
          'right_arm_6_joint: ',round(js[1]['right_arm_6_joint'],2),'\n',\
          'left_arm_0_joint: ',round(js[1]['left_arm_0_joint'],2),'\n',\
          'left_arm_1_joint: ',round(js[1]['left_arm_1_joint'],2),'\n',\
          'left_arm_2_joint: ',round(js[1]['left_arm_2_joint'],2),'\n',\
          'left_arm_3_joint: ',round(js[1]['left_arm_3_joint'],2),'\n',\
          'left_arm_4_joint: ',round(js[1]['left_arm_4_joint'],2),'\n',\
          'left_arm_5_joint: ',round(js[1]['left_arm_5_joint'],2),'\n',\
          'left_arm_6_joint: ',round(js[1]['left_arm_6_joint'],2)
    q = ( 'torso_0_joint', js[1]['torso_0_joint'],
        'right_arm_0_joint',js[1]['right_arm_0_joint'],
        'right_arm_1_joint',js[1]['right_arm_1_joint'],
        'right_arm_2_joint',js[1]['right_arm_2_joint'],
        'right_arm_3_joint',js[1]['right_arm_3_joint'],
        'right_arm_4_joint',js[1]['right_arm_4_joint'],
        'right_arm_5_joint',js[1]['right_arm_5_joint'],
        'right_arm_6_joint',js[1]['right_arm_6_joint'],
        'left_arm_0_joint',js[1]['left_arm_0_joint'],
        'left_arm_1_joint',js[1]['left_arm_1_joint'],
        'left_arm_2_joint',js[1]['left_arm_2_joint'],
        'left_arm_3_joint',js[1]['left_arm_3_joint'],
        'left_arm_4_joint',js[1]['left_arm_4_joint'],
        'left_arm_5_joint',js[1]['left_arm_5_joint'],
        'left_arm_6_joint',js[1]['left_arm_6_joint'],)
    return q



def exitError(code):
    if code == 0:
        print "OK"
        exit(0)
    print "ERROR:", code
    exit(code)

if __name__ == "__main__":
    # define some configurations

    # every joint in position 0
    q_map_0 = {'torso_0_joint':0, 'right_arm_0_joint':0, 'right_arm_1_joint':-1.2,
        'right_arm_2_joint':0, 'right_arm_3_joint':0, 'right_arm_4_joint':0, 'right_arm_5_joint':0,
        'right_arm_6_joint':0, 'left_arm_0_joint':0, 'left_arm_1_joint':0, 'left_arm_2_joint':0,
        'left_arm_3_joint':0, 'left_arm_4_joint':0, 'left_arm_5_joint':0, 'left_arm_6_joint':0 }

    # starting position
    q_map_starting = {'torso_0_joint':0, 
                      'right_arm_0_joint':-0.3, 'left_arm_0_joint':0.3,
                      'right_arm_1_joint':-1.8, 'left_arm_1_joint':1.8,
                      'right_arm_2_joint':1.25, 'left_arm_2_joint':-1.25,
                      'right_arm_3_joint':0.85, 'left_arm_3_joint':-0.85,
                      'right_arm_4_joint':0,    'left_arm_4_joint':0,
                      'right_arm_5_joint':-0.5, 'left_arm_5_joint':0.5,
                      'right_arm_6_joint':0,    'left_arm_6_joint':0 }
        

    # starting position 2
    q_map_starting2 = {'torso_0_joint':0, 
                      'right_arm_0_joint':-0.3, 'left_arm_0_joint':0.3,
                      'right_arm_1_joint':1.0, 'left_arm_1_joint':1.8,
                      'right_arm_2_joint':1.25, 'left_arm_2_joint':-1.25,
                      'right_arm_3_joint':0.85, 'left_arm_3_joint':-0.85,
                      'right_arm_4_joint':0,    'left_arm_4_joint':0,
                      'right_arm_5_joint':-0.5, 'left_arm_5_joint':0.5,
                      'right_arm_6_joint':0,    'left_arm_6_joint':0 }
        


    # goal position
    q_map_goal = {'torso_0_joint':0.0, 
                  'right_arm_0_joint':-1.0, 'left_arm_0_joint':0.3,
                  'right_arm_1_joint':-0.8, 'left_arm_1_joint':1.8,
                  'right_arm_2_joint':0.45, 'left_arm_2_joint':-1.25,
                  'right_arm_3_joint':-0.85,'left_arm_3_joint':-0.85,
                  'right_arm_4_joint':1.5,  'left_arm_4_joint':0,
                  'right_arm_5_joint':1.5,  'left_arm_5_joint':0.5,
                  'right_arm_6_joint':2.0,  'left_arm_6_joint':0 }
           

    # goal position
    q_map_goal2 = {'torso_0_joint':1.0, 'right_arm_0_joint':0, 'right_arm_1_joint':0.0,
        'right_arm_2_joint':0, 'right_arm_3_joint':0, 'right_arm_4_joint':-0.7, 'right_arm_5_joint':0,
        'right_arm_6_joint':0.5, 'left_arm_0_joint':0, 'left_arm_1_joint':0, 'left_arm_2_joint':0,
        'left_arm_3_joint':0, 'left_arm_4_joint':0.2, 'left_arm_5_joint':0, 'left_arm_6_joint':-0.3 }


    # intermediate position
    q_map_intermediate = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':0.0,
        'right_arm_2_joint':-1.25, 'wrong_right_joint':-1.25,'right_arm_3_joint':-0.85, 'right_arm_4_joint':-0.7, 'right_arm_5_joint':-0.5,
        'right_arm_6_joint':0.5, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
        'left_arm_3_joint':-0.85, 'left_arm_4_joint':0.2, 'left_arm_5_joint':0.5, 'left_arm_6_joint':-0.3 }

    rospy.init_node('test_jimp_trapez')

    rospy.sleep(0.5)

    print "This test/tutorial executes simple motions"\
        " in joint trapezoid impedance mode. Planning is not used"\
        " in this example.\n"

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

    rospy.sleep(0.5)

    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(1)

    #print velma.getHandLeftCurrentConfiguration()
    #print velma.getHandRightCurrentConfiguration()

    

    print "Switch to jnt_imp_trapez_mode (no trajectory)..."
    velma.moveJointImpTrapezToCurrentPos(start_time=0.5)
    error = velma.waitForJointTrapez()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(3)

    '''print "Checking if the starting configuration is as expected..."
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_starting, js[1], tolerance=0.3):
        print "This test requires starting pose:"
        print q_map_starting
        exitError(10)'''
    max_vel_2=[0.05, 0.05, 5.5, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
    max_vel_1=[0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
    max_acc_1=[0.225, 0.225, 20.0, 0.225, 0.225, 0.225, 0.225, 0.225, 0.225, 0.225, 0.225, 0.225, 0.225, 0.225, 0.225]
    max_vel=[1.0, 1.0, 1.7, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    max_acc=[0.45, 0.45, 3.35, 0.45, 0.45, 0.45, 0.45, 0.45, 0.45, 0.45, 0.45, 0.45, 0.45, 0.45, 0.45]
    '''print "move to starting position in duration"
    velma.moveJointTrapez(q_map_starting, False, True, False, start_time=0.5, position_tol=15.0/180.0*math.pi, velocity_tol=0)
    error = velma.waitForJointTrapez()
    print error

    print "move to starting_2 position in research duration"
    velma.moveJointTrapez(q_map_starting_2, True, True, True, start_time=0.5, position_tol=15.0/180.0*math.pi, velocity_tol=0)
    error = velma.waitForJointTrapez()
    print "code: ", error

    print "move to starting position in duration"
    velma.moveJointTrapez(q_map_starting, False, True, True, start_time=0.5, position_tol=15.0/180.0*math.pi, velocity_tol=0)
    error = velma.waitForJointTrapez()
    print "code: ", error

    print "Checking if the starting configuration is as expected..."
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_starting, js[1], tolerance=0.01):
        print "This test requires starting pose:"
        print q_map_starting
        exitError(10)'''

    '''traj = JointTrajectory()
    pt = JointTrajectoryPoint()
    for name in q_map_0:
        traj.joint_names.append(name)
        pt.positions.append(q_map_0[name])
        pt.velocities.append(0)
    pt.time_from_start = rospy.Duration(3.0)
    traj.points.append(pt)
    pt = JointTrajectoryPoint()
    for name in q_map_goal2:
        traj.joint_names.append(name)
        pt.positions.append(q_map_goal2[name])
        pt.velocities.append(0)
    pt.time_from_start = rospy.Duration(3.0)
    traj.points.append(pt)
    pt = JointTrajectoryPoint()
    for name in q_map_starting:
        traj.joint_names.append(name)
        pt.positions.append(q_map_starting[name])
        pt.velocities.append(0)
    pt.time_from_start = rospy.Duration(3.0)
    traj.points.append(pt)


    velma.moveJointTrajTrapez(traj, False, True, False, start_time=0.5, position_tol=15.0/180.0 * math.pi, velocity_tol=0)
    error = velma.waitForJointTrapez()
    print "code: ", error'''
    velma.moveJointTrapez(q_map_starting, False, True, False, start_time=0.5, position_tol=15.0/180.0*math.pi, velocity_tol=0)
    error = velma.waitForJointTrapez()
    velma.moveJointTrapez(q_map_goal, False, True, True, start_time=0.5, position_tol=15.0/180.0*math.pi, velocity_tol=0)
    error = velma.waitForJointTrapez()
    #velma.moveJointTrapez(q_map_starting2, True, True, False, start_time=0.5, position_tol=15.0/180.0*math.pi, velocity_tol=0)
    #error = velma.waitForJointTrapez()
    print 
    velma.moveJointTrapez(q_map_starting, False, True, False, start_time=0.5, position_tol=15.0/180.0*math.pi, velocity_tol=0)
    error = velma.waitForJointTrapez()
    velma.moveJointTrapez(q_map_goal2, False, True, True, start_time=0.5, position_tol=15.0/180.0*math.pi, velocity_tol=0)
    error = velma.waitForJointTrapez()
    #velma.moveJointTrapez(q_map_starting2, True, True, False, start_time=0.5, position_tol=15.0/180.0*math.pi, velocity_tol=0)
    #error = velma.waitForJointTrapez()
    print error

    '''print "move to 0 position in duration"
    velma.moveJointTrapez(q_map_goal2, False, True, False, start_time=0.5, position_tol=15.0/180.0*math.pi, velocity_tol=0)
    error = velma.waitForJointTrapez()
    print "code: ", error'''

    showConfig()

    '''print "move to starting_2 position in research velocity 1"
    velma.moveJointTrapez(q_map_goal2, False, False, False,\
     start_time=0.5, position_tol=15.0/180.0*math.pi, velocity_tol=0)
    error = velma.waitForJointTrapez()
    print "code: ", error'''
    #showConfig()
    '''print "move to 0 position in research velocity 2"
    velma.moveJointTrapez(q_map_0, False, False, True ,max_vel,max_acc, start_time=0.5, position_tol=15.0/180.0*math.pi, velocity_tol=0)
    error = velma.waitForJointTrapez()
    print "code: ", error'''
    
    '''print "move to 0 position in duration"
    velma.moveJointTrapez(q_map_starting, False, True, True, start_time=0.5, position_tol=15.0/180.0*math.pi, velocity_tol=0)
    error = velma.waitForJointTrapez()
    print "code: ", error'''

    print "done"

    '''print "Checking if current pose is close to the starting pose..."
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_starting, js[1], tolerance=0.3):
        exitError(10)

    print "waiting 2 seconds..."
    rospy.sleep(2)

    print "Motors must be enabled every time after the robot enters safe state."
    print "If the motors are already enabled, enabling them has no effect."
    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(5)

    print "Moving to position 0 (slowly)."
    velma.moveJointTrapez(q_map_0, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJointTrapez()

    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_0, js[1], tolerance=0.1):
        exitError(10)

    rospy.sleep(1.0)

    print "Switch to jnt_imp mode (no trajectory)..."
    velma.moveJointImpToCurrentPos(start_time=0.5)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(3)

    print "Moving to the starting position..."
    velma.moveJoint(q_map_starting, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(6)

    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_starting, js[1], tolerance=0.1):
        exitError(10)

    print "Moving to valid position, using invalid self-colliding trajectory (this motion should cause error condition, that leads to safe mode in velma_core_cs)."
    velma.moveJoint(q_map_goal, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
        print "The action should have ended with PATH_TOLERANCE_VIOLATED error status, but the error code is", error
        exitError(7)

    print "Using SafeCol behavior to exit self collision..."
    velma.switchToSafeColBehavior()

    print "waiting 2 seconds..."
    rospy.sleep(2)

    print "Motors must be enabled every time after the robot enters safe state."
    print "If the motors are already enabled, enabling them has no effect."
    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(8)

    print "Switch to jnt_imp mode (no trajectory)..."
    velma.moveJointImpToCurrentPos(start_time=0.5)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(3)

    print "Moving to the starting position..."
    velma.moveJoint(q_map_starting, 4.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(9)

    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_starting, js[1], tolerance=0.1):
        exitError(10)'''


