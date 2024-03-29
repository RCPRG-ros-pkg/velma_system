#!/usr/bin/env python

## Provides interactive 1D pose marker that can be used to move individual joints.
# @ingroup utilities
# @file int_markers_jimp.py

import roslib; roslib.load_manifest('velma_common')

import rospy
import math

import xml.dom.minidom as minidom

from std_msgs.msg import ColorRGBA
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from tf.transformations import * 
import tf_conversions.posemath as pm
import PyKDL
import actionlib

from velma_common import VelmaInterface
from rcprg_ros_utils import exitError, marker_publisher

class IntMarkersJimp:
    class JointInfo:
        def __init__(self, name, parent_link, orig, ax, limit_lo, limit_up, rot):
            self.name = name
            self.parent_link = parent_link
            self.orig = orig
            self.ax = ax
            self.limit_lo = limit_lo
            self.limit_up = limit_up
            self.rot = rot

    def getMarkerSize(self, joint_name):
        if joint_name in self.size_map:
            return self.size_map[joint_name]
        return self.default_size

    def __init__(self, velma_interface):
        # Size of individual markers
        self.size_map = {'torso_0_joint':1.0}
        self.default_size = 0.2

        self.__max_vel = rospy.get_param("/max_vel_jnt")
        self.__max_vel_head = rospy.get_param("/max_vel_jnt")

        # Create Velma interface class
        self.velma = velma_interface

        # Get joint infor from robot_description
        robot_description = rospy.get_param("/robot_description")

        # Create marker publisher for joint limits
        self.pub = marker_publisher.MarkerPublisher('/joints_vis')

        # Parse robot_description URDF
        dom = minidom.parseString(robot_description)
        robot = dom.getElementsByTagName("robot")
        if len(robot) != 1:
            print "robot_description does not contain 'robot' node"
            exit(1)

        # Gather information about all joints
        self.joints = []
        for j in robot[0].childNodes:
            if j.localName != "joint":
                continue
            name = j.getAttribute("name")
            joint_type = j.getAttribute("type")
            if joint_type == "fixed":
                continue
            origin = j.getElementsByTagName("origin")
            if len(origin) != 1:
                print "joint '" + name + "' does contain wrong number of 'origin' nodes: " + str(len(origin))
                exit(1)
            axis = j.getElementsByTagName("axis")
            if len(axis) != 1:
                print "joint '" + name + "' contain wrong number of 'axis' nodes: " + str(len(axis))
                exit(1)
            limit = j.getElementsByTagName("limit")
            if len(limit) != 1:
                print "joint '" + name + "' contain wrong number of 'limit' nodes: " + str(len(limit))
                exit(1)
            parent = j.getElementsByTagName("parent")
            if len(parent) != 1:
                print "joint '" + name + "' contain wrong number of 'parent' nodes: " + str(len(parent))
                exit(1)

            xyz = origin[0].getAttribute("xyz")
            rpy = origin[0].getAttribute("rpy")
            xyz = xyz.split()
            rpy = rpy.split()
            orig = PyKDL.Frame(PyKDL.Rotation.RPY(float(rpy[0]), float(rpy[1]), float(rpy[2])), PyKDL.Vector(float(xyz[0]), float(xyz[1]), float(xyz[2])))

            axis_xyz = axis[0].getAttribute("xyz")
            axis_xyz = axis_xyz.split()
            ax = PyKDL.Vector(float(axis_xyz[0]), float(axis_xyz[1]), float(axis_xyz[2]))

            limit_lo = float(limit[0].getAttribute("lower"))
            limit_up = float(limit[0].getAttribute("upper"))
            print name + ": " + str(limit_lo) + " " + str(limit_up) + "  axis: " + str(ax.x()) + " " + str(ax.y()) + " " + str(ax.z()) + " "

            parent_link = parent[0].getAttribute("link")

            axis_z = ax
            if abs(axis_z.z()) > 0.7:
                axis_x = PyKDL.Vector(1,0,0)
            else:
                axis_x = PyKDL.Vector(0,0,1)
            axis_y = axis_z * axis_x
            axis_x = axis_y * axis_z
            axis_x.Normalize()
            axis_y.Normalize()
            axis_z.Normalize()
            rot = PyKDL.Frame(PyKDL.Rotation(axis_z,axis_x,axis_y))

            self.joints.append( IntMarkersJimp.JointInfo(name, parent_link, orig, ax,
                                                                        limit_lo, limit_up, rot) )

        # Create meshes for joint limits visualization
        self.points_list = {}
        for joint in self.joints:
            scale = self.getMarkerSize(joint.name)*0.5
            pl = []
            rot = joint.rot
            angle = joint.limit_lo
            while angle < joint.limit_up-0.15:
                angle_prev = angle
                angle += 0.1
                pl.append(PyKDL.Vector())
                pl.append(rot*PyKDL.Vector(0, math.cos(angle_prev), math.sin(angle_prev))*scale)
                pl.append(rot*PyKDL.Vector(0, math.cos(angle), math.sin(angle))*scale)
            pl.append(PyKDL.Vector())
            pl.append(rot*PyKDL.Vector(0, math.cos(angle), math.sin(angle))*scale)
            pl.append(rot*PyKDL.Vector(0, math.cos(joint.limit_up), math.sin(joint.limit_up))*scale)
            self.points_list[joint.name] = pl

        # Create an interactive marker server on the topic namespace int_markers_jimp
        self.server = InteractiveMarkerServer('int_markers_jimp')
        self.insertMarkers()
        self.server.applyChanges();

    def insertMarkers(self):
        imp_joints = self.velma.getJointGroup('impedance_joints')
        head_joints = self.velma.getJointGroup('head')
        js = self.velma.getLastJointState()[1]

        self.initial_q = {}
        self.prev_q = {}
        self.desired_q = {}
        for joint in self.joints:
            if (not joint.name in imp_joints) and (not joint.name in head_joints):
                continue
            self.initial_q[joint.name] = js[joint.name]
            self.prev_q[joint.name] = js[joint.name]
            self.desired_q[joint.name] = js[joint.name]

            int_position_marker = InteractiveMarker()
            int_position_marker.header.frame_id = joint.parent_link
            int_position_marker.name = joint.name
            int_position_marker.scale = self.getMarkerSize(joint.name)
            int_position_marker.pose = pm.toMsg(joint.orig)

            int_position_marker.controls.append(self.createInteractiveMarkerControl1DOF(joint.rot));

            box = self.createAxisMarkerControl(0.02, self.getMarkerSize(joint.name)*0.5, joint.rot,
                                                                                js[joint.name] )
            box.interaction_mode = InteractiveMarkerControl.BUTTON
            box.name = 'button'
            int_position_marker.controls.append( box )
            self.server.insert(int_position_marker, self.processFeedback);

        self.server.applyChanges();

    def getImpCmd(self):
        joint_names = self.velma.getJointGroup('impedance_joints')
        pos_changed = False
        result = {}
        for joint_name in joint_names:
            if abs(self.desired_q[joint_name] - self.prev_q[joint_name]) > 0.0001:
                pos_changed = True
            result[joint_name] = self.desired_q[joint_name]
        if pos_changed:
            return result
        return None

    def getHeadCmd(self):
        joint_names = self.velma.getJointGroup('head')
        pos_changed = False
        result = {}
        for joint_name in joint_names:
            if abs(self.desired_q[joint_name] - self.prev_q[joint_name]) > 0.0001:
                pos_changed = True
            result[joint_name] = self.desired_q[joint_name]
        if pos_changed:
            return result
        return None

    def processFeedback(self, feedback):
        joint_name = feedback.marker_name
        #print "feedback", feedback.marker_name, feedback.control_name, feedback.event_type
        T_B_Td = pm.fromMsg(feedback.pose)
        rot_diff = PyKDL.diff(PyKDL.Frame(), T_B_Td).rot
        axis = None
        for joint in self.joints:
            if joint_name == joint.name:
                axis = joint.ax
                break
        diff_q = PyKDL.dot(axis, rot_diff)

        self.desired_q[joint.name] = self.initial_q[joint.name] + diff_q
        if ( feedback.control_name == 'button' ) and\
                ( feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK ):
            imp_cmd = self.getImpCmd()
            head_cmd = self.getHeadCmd()
            if not imp_cmd is None:
                self.velma.moveJoint(imp_cmd, None, max_vel=self.__max_vel, start_time=0.5, position_tol=15.0/180.0*math.pi)
            if not head_cmd is None:
                head_dst_q = [head_cmd["head_pan_joint"], head_cmd["head_tilt_joint"]]
                self.velma.moveHead(head_dst_q, None, max_vel=self.__max_vel_head, start_time=0.5, position_tol=15.0/180.0*math.pi)
            for joint_name in self.desired_q:
                self.prev_q[joint_name] = self.desired_q[joint_name]

    def createAxisMarkerControl(self, width, length, rot, init_q):
        marker = Marker()
        marker.type = Marker.ARROW
        marker.scale = Point(width, width*2, width*2)
        marker.color = ColorRGBA(0,1,0,1)
        marker.points.append( Point(0,0,0) )
        v = rot*PyKDL.Vector(0, math.cos(init_q), math.sin(init_q))*length
        marker.points.append( Point(v.x(), v.y(), v.z()) )

        control = InteractiveMarkerControl()
        control.always_visible = True;
        control.markers.append( marker );

        return control

    def createInteractiveMarkerControl1DOF(self, T):
        pose = pm.toMsg(T)
        control = InteractiveMarkerControl()
        control.orientation_mode = InteractiveMarkerControl.FIXED
        control.name = 'control'
        control.orientation = pose.orientation
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        return control

    def spin(self):
        # Visualize the current position and joint limits
        imp_joints = self.velma.getJointGroup('impedance_joints')
        head_joints = self.velma.getJointGroup('head')

        while not rospy.is_shutdown():
            js = self.velma.getLastJointState()[1]
            m_id = 0
            for joint in self.joints:
                if (not joint.name in imp_joints) and (not joint.name in head_joints):
                    continue

                m_id = self.pub.publishTriangleListMarker(self.points_list[joint.name], m_id,
                                                    r=0, g=1, b=0, a=0.5, namespace="limits",
                                                        frame_id=joint.parent_link, T=joint.orig)
                if joint.name in js:
                    q = js[joint.name]
                    vec = joint.rot*PyKDL.Vector(0, math.cos(q), math.sin(q))            
                    m_id = self.pub.publishVectorMarker(joint.orig*PyKDL.Vector(),
                            joint.orig*(vec*0.75*self.getMarkerSize(joint.name)), m_id,
                            1, 0, 0, a=1.0, namespace="limits", frame=joint.parent_link, scale=0.01)
            try:
                rospy.sleep(0.05)
            except:
                break

if __name__ == "__main__":
    rospy.init_node('int_markers_jimp', anonymous=False)

    rospy.sleep(0.5)

    velma = VelmaInterface()
    print "waiting for init..."

    velma.waitForInit()
    print "init ok"

    print "Switch to jnt_imp mode (no trajectory)..."
    if not velma.moveJointImpToCurrentPos(start_time=0.2):
        exitError(1)
    if velma.waitForJoint() != 0:
        exitError(2)

    print "Switched to jnt_imp mode."

    int_jnt = IntMarkersJimp(velma)

    int_jnt.spin()
