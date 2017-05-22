#!/usr/bin/env python

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

#    print "waiting for service /gazebo/unpause_physics"
#    rospy.wait_for_service('/gazebo/unpause_physics')
#    srv_unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', std_srvs.srv.Empty)

    print "waiting for service /gazebo/enable_sim"
    rospy.wait_for_service('/gazebo/enable_sim')
    srv_unpause_physics = rospy.ServiceProxy('/gazebo/enable_sim', std_srvs.srv.Empty)

#    print "waiting for service /velma_core_ve_handright/getSubsystemInfo"
#    rospy.wait_for_service('/velma_core_ve_handright/getSubsystemInfo')
#    srv_info_velma_core_ve_handright = rospy.ServiceProxy('/velma_core_ve_handright/getSubsystemInfo', subsystem_msgs.srv.GetSubsystemInfo)

#    print "waiting for service /velma_core_ve_handleft/getSubsystemInfo"
#    rospy.wait_for_service('/velma_core_ve_handleft/getSubsystemInfo')
#    srv_info_velma_core_ve_handleft = rospy.ServiceProxy('/velma_core_ve_handleft/getSubsystemInfo', subsystem_msgs.srv.GetSubsystemInfo)

    print "waiting for service /velma_core_ve_body/getSubsystemInfo"
    rospy.wait_for_service('/velma_core_ve_body/getSubsystemInfo')
    srv_info_velma_core_ve_body = rospy.ServiceProxy('/velma_core_ve_body/getSubsystemInfo', subsystem_msgs.srv.GetSubsystemInfo)

#    print "waiting for service /velma_core_ve_head/getSubsystemInfo"
#    rospy.wait_for_service('/velma_core_ve_head/getSubsystemInfo')
#    srv_info_velma_core_ve_head = rospy.ServiceProxy('/velma_core_ve_head/getSubsystemInfo', subsystem_msgs.srv.GetSubsystemInfo)

    print "waiting for init"
    while True:
#        info_handright = srv_info_velma_core_ve_handright()
#        info_handleft = srv_info_velma_core_ve_handleft()
        info_body = srv_info_velma_core_ve_body()
#        info_head = srv_info_velma_core_ve_head()

#        if info_handright.is_initialized and info_handleft.is_initialized and info_body.is_initialized and info_head.is_initialized:
        if info_body.is_initialized:
            srv_unpause_physics()
            print "unpaused gazebo"
            break

        time.sleep(1)

