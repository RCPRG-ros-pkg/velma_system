#!/usr/bin/env python

## Script used to unpause the simulation after initialization procedure is complete.
# @ingroup utilities
# @file unpause_on_init.py
# @namespace scripts.unpause_on_init Script used to unpause the simulation after initialization procedure is complete

# Copyright (c) 2015, Robot Control and Pattern Recognition Group,
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

import roslib
roslib.load_manifest('velma_sim_gazebo')

import rospy
import time
import subsystem_msgs.srv
import std_srvs.srv

if __name__ == '__main__':

    rospy.init_node('unpause_on_init')

    print "waiting for service /gazebo/enable_sim"
    rospy.wait_for_service('/gazebo/enable_sim')
    srv_unpause_physics = rospy.ServiceProxy('/gazebo/enable_sim', std_srvs.srv.Empty)

    print "waiting for service /velma_sim_gazebo/getSubsystemInfo"
    rospy.wait_for_service('/velma_sim_gazebo/getSubsystemInfo')
    srv_info_velma_sim_gazebo = rospy.ServiceProxy('/velma_sim_gazebo/getSubsystemInfo', subsystem_msgs.srv.GetSubsystemInfo)

    print "waiting for init"
    while True:
        info_body = srv_info_velma_sim_gazebo()

        if info_body.is_initialized:
            srv_unpause_physics()
            print "unpaused gazebo"
            break

        time.sleep(1)

