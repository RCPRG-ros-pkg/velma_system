#!/usr/bin/env python

# Inverse kinematics for KUKA LWR 4+ robot
# Copyright (c) 2021, Robot Control and Pattern Recognition Group,
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

import rospy
import copy
import cv2
import PyKDL
import math
import numpy as np
import random

from visualization_msgs.msg import *
from sensor_msgs.msg import JointState

from rcprg_ros_utils.marker_publisher import *
from velma_kinematics.velma_ik_geom import KinematicsSolverLWR4, KinematicsSolverVelma

def strFrame(T):
    qx, qy, qz, qw = T.M.GetQuaternion()
    return 'PyKDL.Frame(PyKDL.Rotation.Quaternion({}, {}, {}, {}), PyKDL.Vector({}, {}, {}))'.format(qx, qy, qz, qw, T.p.x(), T.p.y(), T.p.z())

def printFrame(T):
    print strFrame(T)

def testFK(arm_name):
    assert arm_name in ('right', 'left')
    m_pub = MarkerPublisher('velma_ik_geom')
    js_pub = rospy.Publisher('/joint_states', JointState, queue_size=1000)
    rospy.sleep(0.5)

    solv = KinematicsSolverLWR4()
    js_msg = JointState()
    for i in range(7):
        js_msg.name.append('{}_arm_{}_joint'.format(arm_name, i))
        js_msg.position.append(0.0)

    js_msg.name.append('torso_0_joint')
    js_msg.position.append(0.0)

    phase = 0.0
    while not rospy.is_shutdown():
        q = [   math.sin(phase),
                math.sin(phase*1.1),
                math.sin(phase*1.2),
                math.sin(phase*1.3),
                math.sin(phase*1.4),
                math.sin(phase*1.5),
                math.sin(phase*1.6)]
        phase += 0.01

        T_A0_A7 = solv.calculateFk( q )
        m_id = 0
        m_id = m_pub.publishFrameMarker(T_A0_A7, m_id, scale=0.5,
                            frame='calib_{}_arm_base_link'.format(arm_name), namespace='default')
        js_msg.header.stamp = rospy.Time.now()
        for i in range(7):
            js_msg.position[i] = q[i]
        js_pub.publish(js_msg)

        rospy.sleep(0.04)

def generateTestQ():
    limits = [[-2.96, 2.96],
                [-2.09, 2.09],
                [-2.96, 2.96],
                [-2.095, 2.095],
                [-2.96, 2.96],
                [-2.09, 2.09],
                [-2.96, 2.96],]

    result = []
    samples = 4
    for q0 in np.linspace(limits[0][0], limits[0][1], samples, endpoint=True):
        for q1 in np.linspace(limits[1][0], limits[1][1], samples, endpoint=True):
            for q2 in np.linspace(limits[2][0], limits[2][1], samples, endpoint=True):
                for q3 in np.linspace(limits[3][0], limits[3][1], samples, endpoint=True):
                    for q4 in np.linspace(limits[4][0], limits[4][1], samples, endpoint=True):
                        for q5 in np.linspace(limits[5][0], limits[5][1], samples, endpoint=True):
                            for q6 in np.linspace(limits[6][0], limits[6][1], samples, endpoint=True):
                                result.append( (q0, q1, q2, q3, q4, q5, q6) )
    return result

def testIk1():
    solv = KinematicsSolverLWR4()

    samples = generateTestQ()
    samples_count = len(samples)
    print('Number of samples: {}'.format(samples_count))

    for sample_idx, sample in enumerate(samples):
        q = sample
        # Calculate FK
        print('sample: {} / {}'.format(sample_idx, samples_count))
        T_A0_A7d = solv.calculateFk( q )
        #print('  T_A0_A7d: {}'.format(strFrame(T_A0_A7d)))

        for elbow_circle_angle in np.linspace(-math.pi, math.pi, 10, endpoint=True):
            #print('  elbow_circle_angle: {}'.format(elbow_circle_angle))
            iq = solv.calculateIk(T_A0_A7d, elbow_circle_angle, False, False, False)
            T_A0_A7 = solv.calculateFk( iq )
            # compare results
            diff = PyKDL.diff(T_A0_A7, T_A0_A7d, 1.0)
            if diff.vel.Norm() > 0.00001 or diff.rot.Norm() > 0.00001:
                print('ERROR: {}: {}, {}, {}, {}'.format(sample_idx, sample, elbow_circle_angle, strFrame(T_A0_A7d), strFrame(T_A0_A7)))
                return

def testIk2(arm_name, T_A0_A7d, ampl):
    assert arm_name in ('right', 'left')

    m_pub = MarkerPublisher('velma_ik_geom')
    js_pub = rospy.Publisher('/joint_states', JointState, queue_size=1000)
    rospy.sleep(0.5)

    solv = KinematicsSolverLWR4()

    js_msg = JointState()
    for i in range(7):
        js_msg.name.append('{}_arm_{}_joint'.format(arm_name, i))
        js_msg.position.append(0.0)

    js_msg.name.append('torso_0_joint')
    js_msg.position.append(0.0)

    base_link_name = 'calib_{}_arm_base_link'.format(arm_name)
    phase = 0.0
    while not rospy.is_shutdown():
        tx = ampl * math.sin(phase)
        ty = ampl * math.sin(phase*1.1)
        tz = ampl * math.sin(phase*1.2)
        T_A0_A7d2 = PyKDL.Frame(PyKDL.Rotation(), PyKDL.Vector(tx, ty, tz)) * T_A0_A7d
        elbow_circle_angle = phase*1.3
        phase += 0.01

        flip_elbow = (math.sin(phase*1.51) > 0)
        flip_ee = (math.sin(phase*1.93) > 0)
        flip_shoulder = (math.sin(phase*2.73) > 0)
        q = solv.calculateIk(T_A0_A7d2, elbow_circle_angle, flip_shoulder, flip_elbow, flip_ee)
        pt_shoulder = solv.getDebug('pt_shoulder')
        elbow_pt = solv.getDebug('elbow_pt')
        ee_pt = solv.getDebug('ee_pt')
        #T_A0_A6 = solv.getDebug('T_A0_A6')
        m_id = 0
        if not pt_shoulder is None and not ee_pt is None:
            m_id = m_pub.publishVectorMarker(pt_shoulder, ee_pt, m_id, 1, 0, 0, a=1,
                                            frame=base_link_name, namespace='default', scale=0.01)
        if not pt_shoulder is None and not elbow_pt is None:
            m_id = m_pub.publishVectorMarker(pt_shoulder, elbow_pt, m_id, 1, 0, 0, a=1,
                                            frame=base_link_name, namespace='default', scale=0.01)

        m_id = m_pub.publishFrameMarker(T_A0_A7d2, m_id, scale=0.1,
                                                    frame=base_link_name, namespace='default')
        js_msg.header.stamp = rospy.Time.now()
        for i in range(7):
            if q[i] is None:
                js_msg.position[i] = 0.0
            else:
                js_msg.position[i] = q[i]
        js_pub.publish(js_msg)

        rospy.sleep(0.04)

def randomOrientation():
    while True:
        qx = random.gauss(0.0, 1.0)
        qy = random.gauss(0.0, 1.0)
        qz = random.gauss(0.0, 1.0)
        qw = random.gauss(0.0, 1.0)
        q_len = math.sqrt( qx**2 + qy**2 + qz**2 + qw**2 )
        if q_len > 0.001:
            qx /= q_len
            qy /= q_len
            qz /= q_len
            qw /= q_len
            return PyKDL.Rotation.Quaternion(qx, qy, qz, qw)

def testIk3(arm_name):
    assert arm_name in ('right', 'left')

    m_pub = MarkerPublisher('velma_ik_geom')
    js_pub = rospy.Publisher('/joint_states', JointState, queue_size=1000)
    rospy.sleep(0.5)

    flips = []
    for flip_shoulder in (True, False):
        for flip_elbow in (True, False):
            for flip_ee in (True, False):
                flips.append( (flip_shoulder, flip_elbow, flip_ee) )
    solv = KinematicsSolverVelma()

    torso_angle = 0.0

    if arm_name == 'right':
        central_point = PyKDL.Vector( 0.7, -0.7, 1.4 )
    else:
        central_point = PyKDL.Vector( 0.7, 0.7, 1.4 )

    js_msg = JointState()
    for i in range(7):
        js_msg.name.append('{}_arm_{}_joint'.format(arm_name, i))
        js_msg.position.append(0.0)

    js_msg.name.append('torso_0_joint')
    js_msg.position.append(torso_angle)

    base_link_name = 'calib_{}_arm_base_link'.format(arm_name)
    phase = 0.0
    while not rospy.is_shutdown():
        # Get random pose
        T_B_A7d = PyKDL.Frame(randomOrientation(), central_point + PyKDL.Vector(random.uniform(-0.4, 0.4), random.uniform(-0.4, 0.4), random.uniform(-0.4, 0.4)))

        m_id = 0
        m_id = m_pub.publishFrameMarker(T_B_A7d, m_id, scale=0.1,
                                                    frame='world', namespace='default')
        
        for flip_shoulder, flip_elbow, flip_ee in flips:
            for elbow_circle_angle in np.linspace(-math.pi, math.pi, 20):
                arm_q = solv.calculateIkArm(arm_name, T_B_A7d, torso_angle, elbow_circle_angle, flip_shoulder, flip_elbow, flip_ee)

                if not arm_q[0] is None:
                    js_msg.header.stamp = rospy.Time.now()
                    for i in range(7):
                        js_msg.position[i] = arm_q[i]
                    js_pub.publish(js_msg)
                    rospy.sleep(0.04)
                if rospy.is_shutdown():
                    break
            if rospy.is_shutdown():
                break
        rospy.sleep(0.04)


def testIk4flip(arm_name):
    assert arm_name in ('right', 'left')

    m_pub = MarkerPublisher('velma_ik_geom')
    js_pub = rospy.Publisher('/joint_states', JointState, queue_size=1000)
    rospy.sleep(0.5)

    flips = []
    for flip_shoulder in (True, False):
        for flip_elbow in (True, False):
            for flip_ee in (True, False):
                flips.append( (flip_shoulder, flip_elbow, flip_ee) )
    solv = KinematicsSolverVelma()

    torso_angle = 0.0

    if arm_name == 'right':
        central_point = PyKDL.Vector( 0.7, -0.7, 1.4 )
    else:
        central_point = PyKDL.Vector( 0.7, 0.7, 1.4 )

    js_msg = JointState()
    for i in range(7):
        js_msg.name.append('{}_arm_{}_joint'.format(arm_name, i))
        js_msg.position.append(0.0)

    js_msg.name.append('torso_0_joint')
    js_msg.position.append(torso_angle)

    base_link_name = 'calib_{}_arm_base_link'.format(arm_name)
    phase = 0.0
    while not rospy.is_shutdown():
        # Get random pose
        T_B_A7d = PyKDL.Frame(randomOrientation(), central_point + PyKDL.Vector(random.uniform(-0.4, 0.4), random.uniform(-0.4, 0.4), random.uniform(-0.4, 0.4)))

        m_id = 0
        m_id = m_pub.publishFrameMarker(T_B_A7d, m_id, scale=0.1,
                                                    frame='world', namespace='default')
        
        for flip_shoulder, flip_elbow, flip_ee in flips:
            elbow_circle_angle = 0.0
            #for elbow_circle_angle in np.linspace(-math.pi, math.pi, 20):
            arm_q = solv.calculateIkArm(arm_name, T_B_A7d, torso_angle, elbow_circle_angle, flip_shoulder, flip_elbow, flip_ee)

            if not arm_q[0] is None:
                js_msg.header.stamp = rospy.Time.now()
                for i in range(7):
                    js_msg.position[i] = arm_q[i]
                js_pub.publish(js_msg)
                rospy.sleep(0.04)
                rospy.sleep(2.0)
            if rospy.is_shutdown():
                break
            # if rospy.is_shutdown():
            #     break
        rospy.sleep(0.04)

def main():
    rospy.init_node('test_velma_ik_geom', anonymous=False)

    # Requires:
    # roslaunch velma_description upload_robot.launch
    # rosrun robot_state_publisher robot_state_publisher

    v_solv = KinematicsSolverVelma()
    printFrame( v_solv.getLeftArmBaseFk(0.0) )
    printFrame( v_solv.getRightArmBaseFk(0.0) )
    printFrame( v_solv.getLeftArmFk(0.0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) )
    printFrame( v_solv.getRightArmFk(0.0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) )

    #testFK('right')
    #return 0

    #T_A0_A7d = PyKDL.Frame(PyKDL.Rotation.Quaternion(0, 0.00304150858481, 0.0910915674525, 0.995837876145), PyKDL.Vector(0.10227842037159, 0.2623692295165, 0.30143578700507))
    #testIk2( 'left', T_A0_A7d, 0.5 )
    #testIk3( 'left' )
    testIk4flip( 'left' )
    return 0

if __name__ == "__main__":
    exit(main())
