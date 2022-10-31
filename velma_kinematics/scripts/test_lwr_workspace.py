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
import rospkg
import copy
import PyKDL
import math
import numpy as np

from visualization_msgs.msg import *
from sensor_msgs.msg import JointState

from rcprg_ros_utils.marker_publisher import *
from velma_kinematics.velma_workspace import LWRWorkspace

def testLWRWorkspace():
    arm_name = 'right'
    arm_name = 'left'

    m_pub = MarkerPublisher('lwr_workspace')
    js_pub = rospy.Publisher('/joint_states', JointState, queue_size=1000)
    rospy.sleep(0.5)

    rospack = rospkg.RosPack()
    ws_data_path = rospack.get_path('velma_kinematics') + '/data/workspace/'
    ws_param_filename = ws_data_path + 'lwr_ws_param.npy'
    ws_filename = ws_data_path + 'lwr_ws.npy'

    js_msg = JointState()
    for i in range(7):
        js_msg.name.append('{}_arm_{}_joint'.format(arm_name, i))
        js_msg.position.append(0.0)
    js_msg.name.append('torso_0_joint')
    js_msg.position.append(0.0)

    generate = False
    if generate:
        ws = LWRWorkspace.generate()
        ws.save(ws_param_filename, ws_filename)
    else:
        ws = LWRWorkspace.load(ws_param_filename, ws_filename)
        #ws = LWRWorkspace.load()

    x_range = list(range(ws.getCellsX()))
    y_range = list(range(ws.getCellsY()))
    z_range = list(range(ws.getCellsZ()))

    slice_coord = 'y'

    if slice_coord == 'x':
        ic_cells = ws.getCellsX()
    elif slice_coord == 'y':
        ic_cells = ws.getCellsY()
    elif slice_coord == 'z':
        ic_cells = ws.getCellsZ()
    ic = 0

    while not rospy.is_shutdown():
        m_id = 0
        if slice_coord == 'x':
            x_range = [ic]
        elif slice_coord == 'y':
            y_range = [ic]
        elif slice_coord == 'z':
            z_range = [ic]

        for ix in x_range:
            for iy in y_range:
                for iz in z_range:
                    pt = PyKDL.Vector( ws.getCellCenterX(ix), ws.getCellCenterY(iy),
                                                                                ws.getCellCenterZ(iz))
                    col = ws.getCellValue(ix, iy, iz)
                    #col = math.sqrt( ws.getCellValue(ix, iy, iz) )
                    size = ws.getCellSize()
                    m_id = m_pub.addSinglePointMarker(pt, m_id, r=col, g=col, b=col, a=1,
                            namespace='default', frame_id='calib_{}_arm_base_link'.format(arm_name),
                            m_type=Marker.CUBE, scale=Vector3(size, size, size), T=None)

        m_pub.publishAll()
        ic = (ic+1)%ic_cells

        js_msg.header.stamp = rospy.Time.now()
        js_pub.publish(js_msg)

        rospy.sleep(0.5)

    return 0

def main():
    rospy.init_node('test_lwr_workspace', anonymous=False)
    return testLWRWorkspace()

if __name__ == "__main__":
    exit(main())
