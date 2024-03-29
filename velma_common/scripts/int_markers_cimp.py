#!/usr/bin/env python

## Provides interactive 6D pose marker that can be used to move end-effectors.
# @ingroup utilities
# @file int_markers_cimp.py

import roslib; roslib.load_manifest('velma_common')

import rospy
import math

from std_msgs.msg import ColorRGBA
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from tf.transformations import * 
import tf_conversions.posemath as pm
import PyKDL
import actionlib

from velma_common import *
from rcprg_ros_utils import exitError

def makeWrench(lx,ly,lz,rx,ry,rz):
    return PyKDL.Wrench(PyKDL.Vector(lx,ly,lz), PyKDL.Vector(rx,ry,rz))

class IntMarkersCimp:
    def __init__(self, prefix, velma_interface):
        self.prefix = prefix
        self.velma = velma_interface

        # create an interactive marker server on the topic namespace simple_marker
        self.server = InteractiveMarkerServer('int_markers_cimp_' + self.prefix)

        self.insert6DofGlobalMarker()

        self.marker_state = "global"
        self.int_hide_marker = InteractiveMarker()
        self.int_hide_marker.header.frame_id = 'torso_base'
        self.int_hide_marker.name = self.prefix+'_arm_hide_marker';
        self.int_hide_marker.scale = 0.2
        self.int_hide_marker.pose.orientation = Quaternion(0,0,0,1)
        if self.prefix == "right":
            self.int_hide_marker.pose.position = Point(0, -0.5, 0)
            self.hide = self.createButtoMarkerControl(Point(0.15,0.15,0.15), Point(0.0, 0.0, 0.0), ColorRGBA(0,1,0,1) )
        else:
            self.int_hide_marker.pose.position = Point(0, 0.5, 0)
            self.hide = self.createButtoMarkerControl(Point(0.15,0.15,0.15), Point(0.0, 0.0, 0.0), ColorRGBA(1,0,0,1) )
        self.hide.interaction_mode = InteractiveMarkerControl.BUTTON
        self.hide.name = 'button_hide'
        self.int_hide_marker.controls.append( self.hide )
        self.server.insert(self.int_hide_marker, self.processFeedbackHide);
        
        self.server.applyChanges();

    def erase6DofMarker(self):
        self.server.erase(self.prefix+'_arm_position_marker')
        self.server.applyChanges();

    def insert6DofGlobalMarker(self):
        T_B_T = self.velma.getTf("B", "T" + self.prefix)
        int_position_marker = InteractiveMarker()
        int_position_marker.header.frame_id = 'torso_base'
        int_position_marker.name = self.prefix+'_arm_position_marker'
        int_position_marker.scale = 0.2
        int_position_marker.pose = pm.toMsg(T_B_T)

        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.ROTATE_AXIS,'x'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.ROTATE_AXIS,'y'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.ROTATE_AXIS,'z'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.MOVE_AXIS,'x'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.MOVE_AXIS,'y'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.MOVE_AXIS,'z'));

        box = self.createAxisMarkerControl(Point(0.15,0.015,0.015), Point(0.0, 0.0, 0.0) )
        box.interaction_mode = InteractiveMarkerControl.BUTTON
        box.name = 'button'
        int_position_marker.controls.append( box )
        self.server.insert(int_position_marker, self.processFeedback);
        self.server.applyChanges();

    def globalState(self):
        if self.marker_state == "global" or self.marker_state=="local":
           self.erase6DofMarker()

        self.marker_state = "global"
        self.insert6DofGlobalMarker()

    def hiddenState(self):
        if self.marker_state == "global" or self.marker_state=="local":
           self.erase6DofMarker()

        self.marker_state="hidden"

    def processFeedbackHide(self, feedback):
        if ( feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK and feedback.control_name == "button_hide" ):
           if self.marker_state == "global":
               self.hiddenState()
           else:
               self.globalState();

    def processFeedback(self, feedback):
        #print "feedback", feedback.marker_name, feedback.control_name, feedback.event_type

        if ( feedback.marker_name == self.prefix+'_arm_position_marker' ) and ( feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK ) and ( feedback.control_name == "button" ):
            T_B_Td = pm.fromMsg(feedback.pose)
            self.p = pm.toMsg(T_B_Td)
            #self.velma.moveCartImp(self.prefix, [T_B_Td], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5)
            dmp = 0.35
            imp = makeWrench(1500,1500,1500,150,150,150)
            #print 'damping is {}'.format(dmp)
            self.velma.moveCartImp(self.prefix, [T_B_Td], [5.0], None, None, [imp], [5.0], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, damping=PyKDL.Wrench(PyKDL.Vector(dmp, dmp, dmp),PyKDL.Vector(dmp, dmp, dmp)))

    def createSphereMarkerControl(self, scale, position, color):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale = scale
        marker.pose.position = position
        marker.color = color
        control = InteractiveMarkerControl()
        control.always_visible = True;
        control.markers.append( marker );
        return control

    def createBoxMarkerControl(self, scale, position):
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale = scale
        marker.pose.position = position
        marker.color = ColorRGBA(0.5,0.5,0.5,1)
        control = InteractiveMarkerControl()
        control.always_visible = True;
        control.markers.append( marker );
        return control

    def createAxisMarkerControl(self, scale, position):
        markerX = Marker()
        markerX.type = Marker.ARROW
        markerX.scale = scale
        markerX.pose.position = position
        ori = quaternion_about_axis(0, [0, 1 ,0])
        markerX.pose.orientation = Quaternion(ori[0], ori[1], ori[2], ori[3])
        markerX.color = ColorRGBA(1,0,0,1)
        markerY = Marker()
        markerY.type = Marker.ARROW
        markerY.scale = scale
        markerY.pose.position = position
        ori = quaternion_about_axis(math.pi/2.0, [0, 0 ,1])
        markerY.pose.orientation = Quaternion(ori[0], ori[1], ori[2], ori[3])
        markerY.color = ColorRGBA(0,1,0,1)
        markerZ = Marker()
        markerZ.type = Marker.ARROW
        markerZ.scale = scale
        markerZ.pose.position = position
        ori = quaternion_about_axis(-math.pi/2.0, [0, 1 ,0])
        markerZ.pose.orientation = Quaternion(ori[0], ori[1], ori[2], ori[3])
        markerZ.color = ColorRGBA(0,0,1,1)
        control = InteractiveMarkerControl()
        control.always_visible = True;
        control.markers.append( markerX );
        control.markers.append( markerY );
        control.markers.append( markerZ );
        return control

    def createButtoMarkerControl(self, scale, position, color):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale = scale
        marker.pose.position = position
        marker.pose.orientation = Quaternion(0,0,0,1)
        marker.color = color
        control = InteractiveMarkerControl()
        control.always_visible = True;
        control.markers.append( marker );
        return control

    def createInteractiveMarkerControl6DOF(self, mode, axis):
        control = InteractiveMarkerControl()
        control.orientation_mode = InteractiveMarkerControl.FIXED
        if mode == InteractiveMarkerControl.ROTATE_AXIS:
            control.name = 'rotate_' + axis;
        elif mode == InteractiveMarkerControl.MOVE_AXIS:
            control.name = 'move_' + axis;
        else:
            raise Exception('Wrong axis mode: "{}"'.format(mode))
        if axis == 'x':
            control.orientation = Quaternion(1,0,0,1)
        elif axis == 'y':
            control.orientation = Quaternion(0,1,0,1)
        elif axis == 'z':
            control.orientation = Quaternion(0,0,1,1)
        else:
            raise Exception('Wrong axis name: "{}"'.format(axis))
        control.interaction_mode = mode
        return control

if __name__ == "__main__":

    rospy.init_node('int_markers_cimp', anonymous=False)

    rospy.sleep(0.5)

    velma = VelmaInterface()
    print "waiting for init..."

    velma.waitForInit()
    print "init ok"

    print "Switch to cart_imp mode (no trajectory)..."
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        exitError(1)
    if velma.waitForEffectorRight() != 0:
        exitError(2)

    if not velma.moveCartImpLeftCurrentPos(start_time=0.2):
        exitError(3)
    if velma.waitForEffectorLeft() != 0:
        exitError(4)

    print "Switched to cart_imp mode."

    int_right = IntMarkersCimp("right", velma)
    int_left = IntMarkersCimp("left", velma)

    rospy.spin()

