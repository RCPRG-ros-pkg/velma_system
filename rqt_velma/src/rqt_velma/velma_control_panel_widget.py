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
#
# Author: Dawid Seredynski

from __future__ import division
import os
import subprocess
import tempfile
import threading
import math
import time

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot, QRectF, QPointF
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QHBoxLayout,\
    QScrollArea, QGraphicsScene
from python_qt_binding.QtGui import QPixmap, QStandardItemModel, QStandardItem, QPolygonF

import roslib
import rospkg
import rospy
from rospy.exceptions import ROSException

from velma_common import VelmaInterface, symmetricalConfiguration

class VelmaInitThread:
    def __init__(self):
        self.__velma = VelmaInterface()
        self.__is_initialized = False
        self.__is_cancelled = False
        self.__lock = threading.Lock()
        self.__thread = None

    def start(self):
        assert self.__thread is None
        self.__thread = threading.Thread(target=self.__threadFunction)
        self.__thread.start()

    def __threadFunction(self):
        while (not self.isCanceled() and not rospy.is_shutdown()):
            if self.__velma.waitForInit(timeout_s=10.0):
                self.__setInitializedSuccessful()
                break
            else:
                # Reset the interface
                del self.__velma
                self.__velma = VelmaInterface()

    def cancelAndWait(self):
        self.__lock.acquire()
        self.__is_cancelled = True
        self.__lock.release()
        self.__thread.join()

    def __setInitializedSuccessful(self):
        self.__lock.acquire()
        self.__is_initialized = True
        self.__lock.release()

    def isInitialized(self):
        self.__lock.acquire()
        result = self.__is_initialized
        self.__lock.release()
        return result

    def isCanceled(self):
        self.__lock.acquire()
        result = self.__is_cancelled
        self.__lock.release()
        return result

    def getVelmaInterface(self):
        if not self.isInitialized():
            raise Exception("Tried to get VelmaInterface, but it is not ready")
        self.__thread.join()
        return self.__velma

class VelmaCommandThread:
    def __init__(self, velma):
        self.__lock = threading.Lock()
        self.__thread = None
        self.__velma = velma
        self.__queue = []
        self.__errors = []
        self.__is_active = False
        self.__is_cancelled = False

    def start(self):
        assert self.__thread is None
        self.__thread = threading.Thread(target=self.__threadFunction)
        self.__thread.start()

    def __isCanceled(self):
        self.__lock.acquire()
        result = self.__is_cancelled
        self.__lock.release()
        return result

    def cancelAndWait(self):
        self.__lock.acquire()
        self.__is_cancelled = True
        self.__lock.release()
        self.__thread.join()

    def __threadFunction(self):
        self.setActive( True )
        while (not self.__isCanceled() and not rospy.is_shutdown()):
            time.sleep(0.1)
            cmd = self.__getNextCommand()
            if cmd is None:
                continue
            cmd_name = cmd[0]
            if cmd_name == 'runHoming':
                self.__velma.startHomingHP()
                if self.__velma.waitForHP() != 0:
                    self.__setMessage('ERROR: Could not run homing for HP motor')
                    break
                else:
                    self.__setMessage('INFO: Homing of HP motor successful')
                self.__velma.startHomingHT()
                if self.__velma.waitForHT() != 0:
                    self.__setMessage('ERROR: Could not run homing for HT motor')
                    break
                else:
                    self.__setMessage('INFO: Homing of HT motor successful')

            elif cmd_name == 'enableMotors':
                if self.__velma.enableMotors() != 0:
                    self.__setMessage('ERROR: Could not enable motors')
                    break
                else:
                    self.__setMessage('INFO: Enabled motors')

            elif cmd_name == 'switchToJntImp':
                self.__velma.moveJointImpToCurrentPos(start_time=0.5)
                error = self.__velma.waitForJoint()
                if error != 0:
                    self.__setMessage('ERROR: Could not switchToJntImp - error code: {}'.format(error))
                else:
                    self.__setMessage('INFO: switched to jnt_imp')

            elif cmd_name == 'switchToCartImp':
                self.__velma.moveCartImpRightCurrentPos(start_time=0.5)
                error = self.__velma.waitForEffectorRight()
                if error != 0:
                    self.__setMessage('ERROR: Could not switchToCartImp - error code: {}'.format(error))
                else:
                    self.__setMessage('INFO: switched to cart_imp')

            elif cmd_name == 'switchToRelax':
                self.__velma.switchToRelaxBehavior()
                self.__setMessage('INFO: switch to relax')
                time.sleep(0.5)

            elif cmd_name == 'moveToInitialConfiguration':
                q_map_initial = symmetricalConfiguration( {'torso_0_joint':0,
                    'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8, 'right_arm_2_joint':1.25,
                    'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
                    'right_arm_6_joint':0} )

                self.__velma.moveJoint(q_map_initial, None, max_vel=15.0/180.0*math.pi,
                                                start_time=0.5, position_tol=15.0/180.0*math.pi)
                time.sleep(0.5)
                error = self.__velma.waitForJoint()
                if error != 0:
                    self.__setMessage('ERROR: Could not move - error code: {}'.format(error))
                else:
                    self.__setMessage('INFO: moved to the initial configuration')

            elif cmd_name == 'moveHeadTo0':
                self.__velma.moveHead([0.0, 0.0], None, max_vel=15.0/180.0*math.pi,
                                                start_time=0.2, position_tol=5.0/180.0*math.pi)
                time.sleep(0.5)
                error = self.__velma.waitForHead()
                if error != 0:
                    self.__setMessage('ERROR: Could not move head - error code: {}'.format(error))
                else:
                    self.__setMessage('INFO: moved head to the initial configuration')

            elif cmd_name == 'gripperCmd':
                side = cmd[1]
                gr_cmd = cmd[2]
                if gr_cmd == 'open':
                    q = [0, 0, 0, 0]
                    self.__velma.moveHand(side, q, [1, 1, 1, 1], [4000,4000,4000,4000], 1000, hold=False)
                elif gr_cmd == 'close':
                    q = [math.radians(110), math.radians(110), math.radians(110), 0]
                    self.__velma.moveHand(side, q, [1, 1, 1, 1], [4000,4000,4000,4000], 1000, hold=False)
                elif gr_cmd == 'reset':
                    self.__velma.resetHand(side)
                else:
                    raise Exception('Unknown gripper command: "{}"'.format(gr_cmd))
                if self.__velma.waitForHand(side) != 0:
                    exitError(6)
            else:
                raise Exception('Unknown command: {}'.format(cmd_name))

        self.setActive( False )

    def __getNextCommand(self):
        self.__lock.acquire()
        if len(self.__queue) > 1:
            cmd = self.__queue[0]
            self.__queue = self.__queue[1:]
        elif len(self.__queue) == 1:
            cmd = self.__queue[0]
            self.__queue = []
        else:
            cmd = None
        self.__lock.release()
        return cmd

    def __setMessage(self, msg):
        self.__lock.acquire()
        now = rospy.get_rostime()
        ms = int(now.nsecs/1000000)
        self.__errors.append( '{}.{:03d} {}'.format(now.secs, ms, msg) )
        self.__lock.release()

    def getNextMessage(self):
        self.__lock.acquire()
        if len(self.__errors) > 1:
            result = self.__errors[0]
            self.__errors = self.__errors[1:]
        elif len(self.__errors) == 1:
            result = self.__errors[0]
            self.__errors = []
        else:
            result = None
        self.__lock.release()
        return result

    def setActive(self, active):
        self.__lock.acquire()
        self.__is_active = active
        self.__lock.release()

    def isActive(self):
        self.__lock.acquire()
        result = self.__is_active
        self.__lock.release()
        return result

    def __addCommand(self, cmd):
        self.__lock.acquire()
        self.__queue.append( cmd )
        self.__lock.release()

    def __addCommands(self, list_cmd):
        self.__lock.acquire()
        for cmd in list_cmd:
            self.__queue.append( cmd )
        self.__lock.release()

    def initializeRobot(self):
        self.__addCommands( [('enableMotors',), ('runHoming',)] )

    def enableMotors(self):
        self.__addCommand( ('enableMotors',) )

    def switchToJntImp(self):
        self.__addCommand( ('switchToJntImp',) )

    def switchToCartImp(self):
        self.__addCommand( ('switchToCartImp',) )

    def switchToRelax(self):
        self.__addCommand( ('switchToRelax',) )

    def moveToInitialConfiguration(self):
        self.__addCommand( ('moveToInitialConfiguration',) )

    def moveHeadTo0(self):
        self.__addCommand( ('moveHeadTo0',) )

    def gripperCmd(self, side, cmd):
        self.__addCommand( ('gripperCmd', side, cmd) )

from python_qt_binding.QtCore import Qt, Signal, Slot, QRectF, QPointF, QSize, QRect, QPoint
from python_qt_binding.QtWidgets import QDialog, QGraphicsView, QGraphicsScene, QGraphicsPathItem, QGraphicsPolygonItem, QSizePolicy
from python_qt_binding.QtGui import QColor, QPen, QBrush, QPainterPath, QPolygonF, QTransform, QPainter

class JointVis(QGraphicsView):
    def __init__(self, parent=None):
        super (JointVis, self).__init__ (parent)

        self.__lim = None
        self.__pos_rect = None
        self.__pos = None

        self.__good_brush = QBrush(QColor(0,255,0))
        self.__soft_brush = QBrush(QColor(255,200,0))
        self.__hard_brush = QBrush(QColor(255,100,0))
        self.__singularity_soft_brush = QBrush(QColor(0,200,255))
        self.__singularity_hard_brush = QBrush(QColor(0,100,255))

        self.__scene = QGraphicsScene(QRectF(0, 0, 1, 1))
        self.setScene(self.__scene)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setSizePolicy(QSizePolicy(QSizePolicy.Expanding,
                                                 QSizePolicy.Fixed))
        self.setMaximumHeight(10)
        self.setStyleSheet("border: 0px")

    def paintEvent(self, event):
        if not self.__pos is None and not self.__pos_rect is None:
            limit_range = self.__lim[-1] - self.__lim[0]
            self.__pos_rect.setRect( (self.__pos - self.__lim[0]) / limit_range, 0, 2.0/(self.width()-2), 1)
        self.fitInView(self.__scene.sceneRect(), Qt.IgnoreAspectRatio);
        super(JointVis, self).paintEvent(event)

    def setLimits(self, lim):
        if self.__lim is None:
            self.__lim = lim
            soft_lim = 0.26
            limit_range = self.__lim[-1] - self.__lim[0]
            soft_size = soft_lim/limit_range
            self.__scene.addRect(0, 0, soft_size, 1, QPen(Qt.NoPen), self.__soft_brush)
            self.__scene.addRect(1.0-soft_size, 0, soft_size, 1, QPen(Qt.NoPen), self.__soft_brush)

            self.__scene.addRect(soft_size, 0, (self.__lim[1]-self.__lim[0])/limit_range-2*soft_size, 1, QPen(Qt.NoPen), self.__good_brush)

            if len(self.__lim) == 4:
                self.__scene.addRect((self.__lim[1]-self.__lim[0])/limit_range-soft_size, 0, soft_size, 1, QPen(Qt.NoPen), self.__singularity_soft_brush)
                self.__scene.addRect((self.__lim[2]-self.__lim[0])/limit_range, 0, soft_size, 1, QPen(Qt.NoPen), self.__singularity_soft_brush)

                self.__scene.addRect((self.__lim[1]-self.__lim[0])/limit_range, 0, (self.__lim[2]-self.__lim[1])/limit_range, 1, QPen(Qt.NoPen), self.__singularity_hard_brush)

                self.__scene.addRect((self.__lim[2]-self.__lim[0])/limit_range+soft_size, 0, (self.__lim[3]-self.__lim[2])/limit_range-2*soft_size, 1, QPen(Qt.NoPen), self.__good_brush)

            self.__pos_rect = self.__scene.addRect(0, 0, 0.05, 1, QPen(Qt.NoPen), Qt.black)

    def setPosition(self, pos):
        self.__pos = pos
        limit_range = self.__lim[-1] - self.__lim[0]
        self.__pos_rect.setRect( (self.__pos - self.__lim[0]) / limit_range, 0, 2.0/(self.width()-2), 1)
        self.setToolTip('range: ({:.2f}, {:.2f}), pos: {:.2f}'.format(self.__lim[0], self.__lim[-1], pos))

class Joint2Vis(QGraphicsView):
    def __init__(self, parent=None):
        super (Joint2Vis, self).__init__ (parent)

        self.__min_x = None
        self.__max_x = None
        self.__min_y = None
        self.__max_y = None
        self.__pos_rect = None
        self.__pos1 = None
        self.__pos2 = None

        self.__brush_bad = QBrush(QColor(150,0,0))
        self.__brush_good = QBrush(QColor(0,150,0))
        self.__singularity_soft_brush = QBrush(QColor(0,200,255,128))
        self.__singularity_hard_brush = QBrush(QColor(0,100,255,128))

        self.__scene = QGraphicsScene(QRectF(0, 0, 1, 1))
        self.__scene.setBackgroundBrush( self.__brush_bad );
        self.setScene(self.__scene)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setSizePolicy(QSizePolicy(QSizePolicy.Expanding,
                                                 QSizePolicy.Expanding))
        #self.setMaximumHeight(10)
        self.setStyleSheet("border: 0px")

    def __confToScene(self, x, y):
        return  (x - self.__min_x) / (self.__max_x - self.__min_x),\
                (y - self.__min_y) / (self.__max_y - self.__min_y)

    def paintEvent(self, event):
        if not self.__pos1 is None and not self.__pos_rect is None:
            self.__drawPos()
        self.fitInView(self.__scene.sceneRect(), Qt.KeepAspectRatio);
        super(Joint2Vis, self).paintEvent(event)

    def setLimits(self, poly, joint_5_limits):
        if self.__min_x is None:
            self.__poly = poly
            min_x = float('inf')
            max_x = float('-inf')
            min_y = float('inf')
            max_y = float('-inf')
            for idx in range(0, len(self.__poly), 2):
                x = float(self.__poly[idx])
                y = float(self.__poly[idx+1])
                min_x = min(min_x, x)
                max_x = max(max_x, x)
                min_y = min(min_y, y)
                max_y = max(max_y, y)
            self.__min_x = min_x
            self.__max_x = max_x
            self.__min_y = min_y
            self.__max_y = max_y
            limit1_range = self.__max_x - self.__min_x
            limit2_range = self.__max_y - self.__min_y

            self.__qt_poly = QPolygonF()

            for idx in range(0, len(self.__poly), 2):
                idx2 = (idx+2) % len(self.__poly)
                x1, y1 = self.__confToScene( float(self.__poly[idx]), float(self.__poly[idx+1]) )
                x2, y2 = self.__confToScene( float(self.__poly[idx2]), float(self.__poly[idx2+1]) )
                self.__qt_poly.append(QPointF(x1, y1))
            self.__scene.addPolygon(self.__qt_poly, QPen(QColor(255, 0, 0), 0.01, Qt.SolidLine, Qt.RoundCap), self.__brush_good)

            limit_range = joint_5_limits[-1] - joint_5_limits[0]
            soft_lim = 0.26
            soft_size = soft_lim/limit_range
            self.__scene.addRect((joint_5_limits[1]-joint_5_limits[0])/limit_range-soft_size, 0, soft_size, 1, QPen(Qt.NoPen), self.__singularity_soft_brush)
            self.__scene.addRect((joint_5_limits[2]-joint_5_limits[0])/limit_range, 0, soft_size, 1, QPen(Qt.NoPen), self.__singularity_soft_brush)
            self.__scene.addRect((joint_5_limits[1]-joint_5_limits[0])/limit_range, 0, (joint_5_limits[2]-joint_5_limits[1])/limit_range, 1, QPen(Qt.NoPen), self.__singularity_hard_brush)

            self.__pos_point = None

    def setPosition(self, pos1, pos2):
        self.__pos1 = pos1
        self.__pos2 = pos2
        self.__drawPos()

    def __drawPos(self):
        x, y = self.__confToScene( self.__pos1, self.__pos2 )
        if self.__pos_point is None:
            self.__pos_point = (self.__scene.addLine(x-0.03, y-0.03, x+0.03, y+0.03,
                                    QPen(QColor(255, 255, 255), 0.01, Qt.SolidLine, Qt.RoundCap)),
                                self.__scene.addLine(x-0.03, y+0.03, x+0.03, y-0.03,
                                    QPen(QColor(255, 255, 255), 0.01, Qt.SolidLine, Qt.RoundCap)))
        else:
            self.__pos_point[0].setLine(x-0.03, y-0.03, x+0.03, y+0.03)
            self.__pos_point[1].setLine(x-0.03, y+0.03, x+0.03, y-0.03)

    def resizeScene(self):
        self.fitInView(self.__scene.sceneRect(), Qt.KeepAspectRatio)

    def resizeEvent(self, event):
        # call fitInView each time the widget is resized
        self.resizeScene()

    def showEvent(self, event):
        # call fitInView each time the widget is shown
        self.resizeScene()
    

class VelmaControlPanelWidget(QWidget):
    """
    main class inherits from the ui window class.

    You can specify the topics that the topic pane.

    VelmaControlPanelWidget.start must be called in order to update topic pane.
    """

    _column_names = ['topic', 'type', 'bandwidth', 'rate', 'value']

    def __init__(self, plugin=None):
        """
        """
        super(VelmaControlPanelWidget, self).__init__()

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_velma'), 'resource', 'VelmaControlPanelWidget.ui')
        loadUi(ui_file, self)

        self._plugin = plugin

        self.__list_model = QStandardItemModel()
        self.list_messages.setModel(self.__list_model)

        #self.button_initialize_robot.clicked.connect(
        #        lambda: self.__velma_cmd.initializeRobot() if not self.__velma_cmd is None)
        self.button_initialize_robot.clicked.connect( self.clicked_initialize_robot )
        self.button_enable_motors.clicked.connect( self.clicked_enable_motors )
        self.button_switch_to_relax.clicked.connect( self.clicked_switch_to_relax )
        self.button_switch_to_cart_imp.clicked.connect( self.clicked_switch_to_cart_imp )
        self.button_switch_to_jnt_imp.clicked.connect( self.clicked_switch_to_jnt_imp )
        self.button_clear_messages.clicked.connect( self.clicked_clear_messages )
        self.button_move_to_initial_configuration.clicked.connect( self.clicked_move_to_initial_configuration )
        self.button_move_head_to_0.clicked.connect( self.clicked_move_head_to_0 )

        self.button_right_gripper_open.clicked.connect( lambda: self.gripper_cmd('right', 'open') )
        self.button_right_gripper_close.clicked.connect( lambda: self.gripper_cmd('right', 'close') )
        self.button_right_gripper_reset.clicked.connect( lambda: self.gripper_cmd('right', 'reset') )
        self.button_left_gripper_open.clicked.connect( lambda: self.gripper_cmd('left', 'open') )
        self.button_left_gripper_close.clicked.connect( lambda: self.gripper_cmd('left', 'close') )
        self.button_left_gripper_reset.clicked.connect( lambda: self.gripper_cmd('left', 'reset') )

        self.__velma_init = VelmaInitThread()
        self.__velma_init.start()
        self.__velma_cmd = None
        self.__velma = None

        # init and start update timer
        self._timer_refresh_topics = QTimer(self)
        self._timer_refresh_topics.timeout.connect(self.refresh_topics)

        self.__joint_vis_map = None

    def clicked_initialize_robot(self):
        if not self.__velma_cmd is None:
            self.__velma_cmd.initializeRobot()

    def clicked_enable_motors(self):
        if not self.__velma_cmd is None:
            self.__velma_cmd.enableMotors()

    def clicked_switch_to_jnt_imp(self):
        if not self.__velma_cmd is None:
            self.__velma_cmd.switchToJntImp()

    def clicked_switch_to_cart_imp(self):
        if not self.__velma_cmd is None:
            self.__velma_cmd.switchToCartImp()

    def clicked_switch_to_relax(self):
        if not self.__velma_cmd is None:
            self.__velma_cmd.switchToRelax()

    def clicked_move_to_initial_configuration(self):
        if not self.__velma_cmd is None:
            self.__velma_cmd.moveToInitialConfiguration()

    def clicked_move_head_to_0(self):
        if not self.__velma_cmd is None:
            self.__velma_cmd.moveHeadTo0()

    def clicked_clear_messages(self):
        #self.list_messages
        #self.button_clear_messages.
        pass

    def gripper_cmd(self, side, cmd):
        if not self.__velma_cmd is None:
            self.__velma_cmd.gripperCmd(side, cmd)

    def start(self):
        """
        This method needs to be called to start updating topic pane.
        """
        self._timer_refresh_topics.start(100)

    def layout_widgets(self, layout):
       return (layout.itemAt(i) for i in range(layout.count()))

    @Slot()
    def refresh_topics(self):
        if self.__velma is None:
            print(type(self.verticalLayout_4))
            print(self.verticalLayout_4)
            print(dir(self.verticalLayout_4))
            if self.__velma_init.isInitialized():
                self.__velma = self.__velma_init.getVelmaInterface()
                self.__velma_cmd = VelmaCommandThread(self.__velma)
                self.__velma_cmd.start()
                self.__joint_vis_map = {}
                self.__other_widgets = []
                for idx in range(7):
                    left_name = 'left_arm_{}_joint'.format(idx)

                    self.__joint_vis_map[left_name] = JointVis()
                    #self.verticalLayout_4.insertWidget(idx+1, self.__joint_vis_map[left_name])

                    h_layout = QHBoxLayout()
                    label = QLabel()
                    label.setText('{}'.format(idx))
                    h_layout.addWidget( label )
                    h_layout.addWidget( self.__joint_vis_map[left_name] )
                    self.__other_widgets.append(h_layout)
                    self.__other_widgets.append(label)
                    self.verticalLayout_4.insertLayout(idx+1, h_layout)

                    right_name = 'right_arm_{}_joint'.format(idx)
                    self.__joint_vis_map[right_name] = JointVis()
#                    self.verticalLayout_5.insertWidget(idx+1, self.__joint_vis_map[right_name])
                    h_layout = QHBoxLayout()
                    label = QLabel()
                    label.setText('{}'.format(idx))
                    h_layout.addWidget( label )
                    h_layout.addWidget( self.__joint_vis_map[right_name] )
                    self.__other_widgets.append(h_layout)
                    self.__other_widgets.append(label)
                    self.verticalLayout_5.insertLayout(idx+1, h_layout)

                left_name = 'left_arm_double_joint'

                self.__joint_vis_map[left_name] = Joint2Vis()
                self.verticalLayout_4.insertWidget(9, self.__joint_vis_map[left_name])

                right_name = 'right_arm_double_joint'
                self.__joint_vis_map[right_name] = Joint2Vis()
                self.verticalLayout_5.insertWidget(9, self.__joint_vis_map[right_name])



                self.__joint_vis_map['torso_0_joint'] = JointVis()
                self.gridLayout_2.addWidget(self.__joint_vis_map['torso_0_joint'], 0, 1)

                self.__joint_vis_map['head_pan_joint'] = JointVis()
                self.gridLayout_2.addWidget(self.__joint_vis_map['head_pan_joint'], 1, 1)

                self.__joint_vis_map['head_tilt_joint'] = JointVis()
                self.gridLayout_2.addWidget(self.__joint_vis_map['head_tilt_joint'], 2, 1)

                self.__jnt_lim_cart = self.__velma.getCartImpJointLimits()
                self.__head_limits = self.__velma.getHeadJointLimits()
                for joint_name in self.__joint_vis_map:
                    if joint_name in self.__jnt_lim_cart:
                        self.__joint_vis_map[joint_name].setLimits( self.__jnt_lim_cart[joint_name] )
                    elif joint_name in self.__head_limits:
                        self.__joint_vis_map[joint_name].setLimits( self.__head_limits[joint_name] )
                    else:
                        print('WARNING: could not get limits for joint "{}"'.format(joint_name))

                self.__joint_vis_map['left_arm_double_joint'].setLimits( self.__velma.getLeftWccPolygon(), self.__jnt_lim_cart['left_arm_5_joint'] )
                self.__joint_vis_map['right_arm_double_joint'].setLimits( self.__velma.getRightWccPolygon(), self.__jnt_lim_cart['right_arm_5_joint'] )

            self.label_panel_state.setText('Waiting for initialization of Velma Interface')
            self.label_current_state_core_cs.setText( 'unknown' )
            self.label_motors_ready.setText('motors state is unknown')
            self.label_current_state_core_ve_body.setText( 'unknown' )
        else:
            self.label_panel_state.setText('Velma Interface is initialized')

            diag_cs = self.__velma.getCoreCsDiag(timeout_s=1.0)
            self.label_current_state_core_cs.setText( diag_cs.getCurrentStateName() )
            if diag_cs.motorsReady():
                self.label_motors_ready.setText('motors are ready')
            else:
                self.label_motors_ready.setText('motors are not ready')

            diag_ve_body = self.__velma.getCoreVeDiag(timeout_s=1.0)
            self.label_current_state_core_ve_body.setText( diag_ve_body.getCurrentStateModeName() )

            while True:
                msg = self.__velma_cmd.getNextMessage()
                if msg is None:
                    break
                item = QStandardItem(msg)
                self.__list_model.appendRow(item)
            self.list_messages.scrollToBottom()


            js = self.__velma.getLastJointState()
            if not js is None:
                js = js[1]
                for joint_name in self.__joint_vis_map:
                    if joint_name in js:
                        self.__joint_vis_map[joint_name].setPosition( js[joint_name] )

                self.__joint_vis_map['left_arm_double_joint'].setPosition( js['left_arm_5_joint'], js['left_arm_6_joint'] )
                self.__joint_vis_map['right_arm_double_joint'].setPosition( js['right_arm_5_joint'], js['right_arm_6_joint'] )

    def shutdown_plugin(self):
        if not self.__velma_init is None:
            self.__velma_init.cancelAndWait()
        if not self.__velma_cmd is None:
            self.__velma_cmd.cancelAndWait()
        self._timer_refresh_topics.stop()

    # TODO(Enhancement) Save/Restore tree expansion state
    def save_settings(self, plugin_settings, instance_settings):
        header_state = self.topics_tree_widget.header().saveState()
        instance_settings.set_value('tree_widget_header_state', header_state)

    def restore_settings(self, pluggin_settings, instance_settings):
        if instance_settings.contains('tree_widget_header_state'):
            header_state = instance_settings.value('tree_widget_header_state')
            if not self.topics_tree_widget.header().restoreState(header_state):
                rospy.logwarn("rqt_topic: Failed to restore header state.")
