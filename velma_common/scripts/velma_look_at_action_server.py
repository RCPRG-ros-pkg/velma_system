#!/usr/bin/env python

## A ROS node that implements 'look at' action server for head of WUT Velma robot.
# @ingroup utilities
# @file velma_look_at_action_server.py

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

import math

import rospy
import actionlib
import geometry_msgs.msg

from velma_look_at_action_msgs.msg import LookAtAction, LookAtFeedback, LookAtResult
from velma_common.velma_interface import VelmaInterface

class LinearIntervalFunction:
    def __init__(self, points=None):
        self.__points__ = []

        if not points is None:
            for pt in points:
                self.addPoint( pt[0], pt[1] )
    
    def addPoint(self, x, val):
        if len(self.__points__) == 0:
            self.__points__.append( (x, val) )
            return

        for idx, pt in enumerate(self.__points__):
            if x < pt[0]:
                self.__points__.insert( idx, (x, val) )
                break
            elif idx == len(self.__points__)-1:
                self.__points__.append( (x, val) )
                break
            elif x > pt[0] and x < self.__points__[idx+1][0]:
                self.__points__.insert( idx+1, (x, val) )
                break;
            elif x == pt[0] or x == self.__points__[idx+1][0]:
                raise Exception('Added point with the same x twice')

    def interpolate(self, x):
        x = float(x)
        if len(self.__points__) < 2:
            raise Exception('Could not interpolate the function with only one point')

        if x < self.__points__[0][0]:
            raise Exception('x is below the domain range')

        for idx, pt in enumerate(self.__points__):
            if idx == len(self.__points__)-1:
                break
            elif x >= pt[0] and x <= self.__points__[idx+1][0]:
                f = (x - pt[0]) / (self.__points__[idx+1][0] - pt[0])
                return (1.0-f) * pt[1] + f * self.__points__[idx+1][1]
        raise Exception('x is above the domain range: ' + str(x) + ', points: ' + str(self.__points__))

def simplifiedHeadIK(torso_angle, pt):
    # yaw is a sum of torso angle and head pan angle
    yaw = math.atan2( pt.y, pt.x ) - torso_angle

    # pitch is head tilt angle
    xy_dist = math.sqrt( pt.x**2 + pt.y**2 )
    head_z = 1.75
    pitch = -math.atan2( pt.z - head_z, xy_dist )

    return yaw, pitch

def calculateViewQuality(head_q):
    horizontal_fov = 1.047
    horizontal_fov_margin = math.radians(10.0)
    head_pan_limits = [-1.56, 1.56]
    func = LinearIntervalFunction( [
        (-10000, 0),
        (head_pan_limits[0] - horizontal_fov + horizontal_fov_margin, 0),
        (head_pan_limits[0], 1),
        (head_pan_limits[1], 1),
        (head_pan_limits[1] + horizontal_fov - horizontal_fov_margin, 0),
        (10000, 0)] )
    return func.interpolate( head_q[0] )

class VelmaLookAtAction(object):
    # create messages that are used to publish feedback/result
    __feedback = LookAtFeedback()
    __result = LookAtResult()

    def __init__(self, name):
        self.__action_name = name
        self.__head_pan_limits = [-1.56, 1.56]

        self.__velma = VelmaInterface()
        print('VelmaLookAtAction: Waiting for VelmaInterface initialization...')
        if not self.__velma.waitForInit(timeout_s=10.0):
            self.__velma = None
            print('VelmaLookAtAction: Could not initialize VelmaInterface')
            return
            #raise Exception('Could not initialize VelmaInterface')
        print('VelmaLookAtAction: Initialization of VelmaInterface ok!')

        self.__as = actionlib.SimpleActionServer(self.__action_name, LookAtAction,
                        execute_cb=self.execute_cb, auto_start=False)

        #self.__as = actionlib.SimpleActionServer(self.__action_name, LookAtAction,
        #                goal_cb=self.goal_cb, cancel_cb=self.cancel_cb, auto_start=False)

        self.__as.start()
        print('VelmaLookAtAction: Initialization of action server ok!')
        print('VelmaLookAtAction: Action name: {}'.format(self.__action_name))

    def isOk(self):
        return not self.__velma is None

    #def __del__(self):
    #    with self.terminate_mutex:
    #        self.need_to_terminate = True
    #    assert(self.execute_thread)
    #    self.execute_thread.join()

    #def cancel_cb(self):
    #    pass

    def execute_cb(self, goal):
        print('VelmaLookAtAction: Received a new goal: {} ({}, {}, {})'.format(goal.frame_id,
                goal.target.x, goal.target.y, goal.target.z))
        # TODO: Transform target point to the torso_base frame
        assert goal.frame_id == 'torso_base'

        js = self.__velma.getLastJointState()

        # Simple version of IK for head
        head_q = list( simplifiedHeadIK( js[1]['torso_0_joint'] , goal.target ) )

        quality = calculateViewQuality( head_q )
        if head_q[0] < self.__head_pan_limits[0]:
            head_q[0] = self.__head_pan_limits[0]
        elif head_q[0] > self.__head_pan_limits[1]:
            head_q[0] = self.__head_pan_limits[1]

        if quality < 0.001:
            self.__result.error_code = LookAtResult.OUT_OF_RANGE
            print('VelmaLookAtAction: rejected goal: OUT_OF_RANGE')
            self.__as.set_preempted(self.__result)
            return

        diag = self.__velma.getCoreCsDiag()
        if (not diag.inStateCartImp()) and (not diag.inStateJntImp()):
            self.__result.error_code = LookAtResult.WRONG_BEHAVIOUR
            print('VelmaLookAtAction: rejected goal: WRONG_BEHAVIOUR')
            self.__as.set_preempted(self.__result)
            return

        self.__feedback.quality = quality
        # TODO:
        #self.__feedback.view_x_factor
        #self.__feedback.view_y_factor

        dist = max( abs( js[1]['head_pan_joint']-head_q[0] ), abs(js[1]['head_tilt_joint']-head_q[1]) )
        max_vel = 0.5   # radians per second
        time = dist / max_vel
        self.__velma.moveHead(head_q, time, start_time=0.5)
        while not rospy.is_shutdown():
            result = self.__velma.waitForHead(timeout_s=0.1)
            if not result is None:
                if result == 0:
                    break
                else:
                    self.__result.error_code = LookAtResult.MOTION_FAILED
                    print('VelmaLookAtAction: aborted: MOTION_FAILED')
                    self.__as.set_preempted(self.__result)
                    return
            # check that preempt has not been requested by the client
            if self.__as.is_preempt_requested():
                # set the action state to preempted
                js = self.__velma.getLastJointState()
                print "js: ", js[1]['head_pan_joint'],js[1]['head_tilt_joint']
                max_vel = 0.5   # radians per second
                time = 0.1
                self.__velma.moveHead((js[1]['head_pan_joint'],js[1]['head_tilt_joint']), time, start_time=0.01)
                while not rospy.is_shutdown():
                    result = self.__velma.waitForHead(timeout_s=0.1)
                    if not result is None:
                        if result == 0:
                            break
                        else:
                            self.__result.error_code = LookAtResult.MOTION_FAILED
                            print('VelmaLookAtAction: aborted: MOTION_FAILED')
                            self.__as.set_preempted(self.__result)
                            return
                self.__result.error_code = LookAtResult.CANCELLED
                self.__as.set_preempted(self.__result)
                return
            
            self.__as.publish_feedback(self.__feedback)

        self.__result.error_code = LookAtResult.SUCCESSFUL
        print('VelmaLookAtAction: reached the goal: SUCCESSFUL')
        self.__as.set_succeeded(self.__result)

if __name__ == '__main__':

    ## Test
    #torso_q = -0.5
    #head_q = simplifiedHeadIK(torso_q, geometry_msgs.msg.Point(0.25,-1,1))
    #quality = calculateViewQuality(torso_q, head_q)
    #print( 'head_q: {}, {}; quality: {}'.format(head_q[0], head_q[1], quality) )
    #exit(0)

    rospy.init_node('velma_look_at_action')
    server = VelmaLookAtAction('velma_look_at_action')
    if server.isOk():
        rospy.spin()
        exit(0)
    else:
        exit(1)
