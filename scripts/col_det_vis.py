#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_core_cs')

import sys
import rospy
import math
import copy
import tf

import xml.dom.minidom as minidom
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from tf.transformations import * 
import tf_conversions.posemath as pm
import PyKDL
from diagnostic_msgs.msg import *

from velma_common.velma_interface import *
import velma_common.velmautils as velmautils

class Geometry(object):
    def __init__(self, type_name):
        self.f = None
        self.type = type_name

class Capsule(Geometry):
    def __init__(self):
        super(Capsule, self).__init__("CAPSULE")
        self.r = None
        self.l = None

class Sphere(Geometry):
    def __init__(self):
        super(Sphere, self).__init__("SPHERE")
        self.r = None

class Link:
    def __init__(self):
        self.idx = None
        self.name = None
        self.geoms = []


class ColDetVis:
    def __init__(self):
        self.pub_marker = velmautils.MarkerPublisher()
        rospy.Subscriber("/velma_core_cs/diag", DiagnosticArray, self.diagCallback)
        self.links = {}

    def diagCallback(self, data):
        m_id = 0
        for v in data.status[1].values:
            if v.key == "ColDet":
                dom = minidom.parseString(v.value)
                cd = dom.getElementsByTagName("cd")
                col_count = cd[0].getAttribute("col_count")
#                print col_count

                cols = cd[0].getElementsByTagName("c")

                if len(cols) > 0:
                    print "collisions:"

                for c in cols:
                    i1 = int(c.getAttribute("i1"))
                    i2 = int(c.getAttribute("i2"))
                    p1x = float(c.getAttribute("p1x"))
                    p1y = float(c.getAttribute("p1y"))
                    p1z = float(c.getAttribute("p1z"))
                    p2x = float(c.getAttribute("p2x"))
                    p2y = float(c.getAttribute("p2y"))
                    p2z = float(c.getAttribute("p2z"))
                    d = float(c.getAttribute("d"))
                    n1x = float(c.getAttribute("n1x"))
                    n1y = float(c.getAttribute("n1y"))
                    n1z = float(c.getAttribute("n1z"))
                    n2x = float(c.getAttribute("n2x"))
                    n2y = float(c.getAttribute("n2y"))
                    n2z = float(c.getAttribute("n2z"))

                    v1 = PyKDL.Vector(p1x, p1y, p1z)
                    v2 = PyKDL.Vector(p2x, p2y, p2z)
                    m_id = self.pub_marker.publishVectorMarker(v1, v2, m_id, 1, 0, 0, frame='torso_base', namespace='default', scale=0.01)
                    if i1 in self.links and i2 in self.links:
                        print self.links[i1].name, self.links[i2].name

                links = cd[0].getElementsByTagName("l")
                for l in links:
                    idx = int(l.getAttribute("idx"))
                    if not idx in self.links:
                        self.links[idx] = Link()
                        self.links[idx].idx = idx
                        self.links[idx].name = l.getAttribute("name")
                        geoms = l.getElementsByTagName("g")
                        for g in geoms:
                            x = float(g.getAttribute("x"))
                            y = float(g.getAttribute("y"))
                            z = float(g.getAttribute("z"))
                            rx = float(g.getAttribute("rx"))
                            ry = float(g.getAttribute("ry"))
                            rz = float(g.getAttribute("rz"))
                            rot = PyKDL.Vector(rx, ry, rz)
                            rot_angle = rot.Normalize()
                            f = PyKDL.Frame(PyKDL.Rotation.Rot(rot, rot_angle), PyKDL.Vector(x,y,z))
                            type_name = g.getAttribute("type")
                            if type_name == "SPHERE":
                                gg = Sphere()
                                gg.r = float(g.getAttribute("r"))
                            elif type_name == "CAPSULE":
                                gg = Capsule()
                                gg.r = float(g.getAttribute("r"))
                                gg.l = float(g.getAttribute("l"))
                            else:
                                gg = None
                        if gg:
                            gg.f = f
                            self.links[idx].geoms.append(gg)

                self.pub_marker.eraseMarkers(m_id, 200, frame_id='torso_base', namespace='default')
                break

        m_id = 300
        for idx in self.links:
            link = self.links[idx]
            for g in link.geoms:
                if g.type == "SPHERE":
                    m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(), m_id, r=0, g=1, b=0, a=0.5, namespace='collision', frame_id=link.name, m_type=Marker.SPHERE, scale=Vector3(g.r*2, g.r*2, g.r*2), T=g.f)
                if g.type == "CAPSULE":
                    m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(0,0,-g.l/2), m_id, r=0, g=1, b=0, a=0.5, namespace='collision', frame_id=link.name, m_type=Marker.SPHERE, scale=Vector3(g.r*2, g.r*2, g.r*2), T=g.f)
                    m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(0,0,g.l/2), m_id, r=0, g=1, b=0, a=0.5, namespace='collision', frame_id=link.name, m_type=Marker.SPHERE, scale=Vector3(g.r*2, g.r*2, g.r*2), T=g.f)
                    m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(), m_id, r=0, g=1, b=0, a=0.5, namespace='collision', frame_id=link.name, m_type=Marker.CYLINDER, scale=Vector3(g.r*2, g.r*2, g.l), T=g.f)


if __name__ == "__main__":

    rospy.init_node('col_det_vis', anonymous=True)

    rospy.sleep(1)

    cdv = ColDetVis()

    rospy.spin()

