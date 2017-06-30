#!/usr/bin/env python

# Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
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

import roslib
roslib.load_manifest('velma_common')

import rospy

import sensor_msgs.msg
import geometry_msgs.msg

import tf
from tf import *
from tf.transformations import * 
import tf_conversions.posemath as pm
from tf2_msgs.msg import *

import PyKDL
import math
import numpy as np

#import copy

#import urdf_parser_py.urdf

class CameraFake:

    def __init__(self):
        pass

    def spin(self):

        # initialize publishers
        pub_cam_info = rospy.Publisher("/fake_camera/camera_info", sensor_msgs.msg.CameraInfo, queue_size=100)
        pub_cam_img = rospy.Publisher("/fake_camera/image", sensor_msgs.msg.Image, queue_size=100)
        self.br = tf.TransformBroadcaster()

        # initialize topic messages
        cam_img = sensor_msgs.msg.Image()
        cam_img.header.seq = 0
        cam_img.header.frame_id = "fake_camera"
        cam_img.height = 1
        cam_img.width = 1
        cam_img.encoding = "mono8"
        cam_img.is_bigendian = 1
        cam_img.step = 1
        cam_img.data = [0]

        cam_info = sensor_msgs.msg.CameraInfo()
        cam_info.header.seq = cam_img.header.seq
        cam_info.header.frame_id = cam_img.header.frame_id
        cam_info.height = 1
        cam_info.width = 1
        cam_info.distortion_model = "plumb_bob"
        cam_info.D = [0, 0, 0, 0, 0]
        fx = 0.4
        fy = 0.4
        cx = 0.5
        cy = 0.5
        cam_info.K = [fx, 0, cx,
                      0, fy, cy,
                      0,  0,  1]

        cam_info.R = [1, 0, 0,
                      0, 1, 0,
                      0, 0, 1]

        cam_info.P = [fx, 0, cx, 0,
                      0, fy, cy, 0,
                      0,  0,  1, 0]

        time_diff = 0.05
        print "fake camera is running"

        while not rospy.is_shutdown():
            time_now = rospy.Time.now()

            cam_img.header.seq += 1
            cam_img.header.stamp = time_now

            cam_info.header.seq = cam_img.header.seq
            cam_info.header.stamp = time_now

            cam_pos = PyKDL.Vector(0.6, 0.6, 2.2)
            cam_target = PyKDL.Vector(0.6, 0.0, 1.2)
            # origin of frame should be optical center of cameara
            # +x should point to the right in the image
            # +y should point down in the image
            # +z should point into to plane of the image
            cam_z = cam_target - cam_pos
            cam_y = PyKDL.Vector(0, 0, -1)
            cam_x = cam_y * cam_z
            cam_y = cam_z * cam_x
            cam_x.Normalize()
            cam_y.Normalize()
            cam_z.Normalize()
            T_W_CAM = PyKDL.Frame(PyKDL.Rotation(cam_x, cam_y, cam_z), cam_pos)
            q = T_W_CAM.M.GetQuaternion()
            p = T_W_CAM.p



            self.br.sendTransform([p[0], p[1], p[2]], [q[0], q[1], q[2], q[3]], time_now, cam_img.header.frame_id, "world")
            pub_cam_img.publish(cam_img)
            pub_cam_info.publish(cam_info)

            rospy.sleep(time_diff)

if __name__ == '__main__':
    rospy.init_node('camera_fake')
    cf = CameraFake()
    cf.spin()

