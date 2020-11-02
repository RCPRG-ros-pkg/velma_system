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

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot, QRectF
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QHBoxLayout,\
    QScrollArea, QGraphicsScene
from python_qt_binding.QtGui import QPixmap, QStandardItemModel, QStandardItem

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
            rospy.sleep(0.1)
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

            elif cmd_name == 'switchToSafeCol':
                self.__velma.switchToSafeColBehavior()
                self.__setMessage('INFO: switched to safe_col')
                rospy.sleep(0.5)

            elif cmd_name == 'moveToInitialConfiguration':
                q_map_starting = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
                    'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
                    'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
                    'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }

                self.__velma.moveJoint(q_map_starting, None, max_vel=15.0/180.0*math.pi, start_time=0.5, position_tol=15.0/180.0*math.pi)
                error = self.__velma.waitForJoint()
                if error != 0:
                    self.__setMessage('ERROR: Could not move - error code: {}'.format(error))
                else:
                    self.__setMessage('INFO: moved to the starting configuration')

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

    def switchToSafeCol(self):
        self.__addCommand( ('switchToSafeCol',) )

    def moveToInitialConfiguration(self):
        self.__addCommand( ('moveToInitialConfiguration',) )

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

        #for i in entries:


        self.button_initialize_robot.clicked.connect( self.clicked_initialize_robot)
        self.button_enable_motors.clicked.connect( self.clicked_enable_motors )
        self.button_switch_to_safe_col.clicked.connect( self.clicked_switch_to_safe_col )
        self.button_switch_to_cart_imp.clicked.connect( self.clicked_switch_to_cart_imp )
        self.button_switch_to_jnt_imp.clicked.connect( self.clicked_switch_to_jnt_imp )
        self.button_clear_messages.clicked.connect( self.clicked_clear_messages )
        self.button_move_to_initial_configuration.clicked.connect( self.clicked_move_to_initial_configuration )

        self.__velma_init = VelmaInitThread()
        self.__velma_init.start()
        self.__velma_command = None
        self.__velma = None

        # init and start update timer
        self._timer_refresh_topics = QTimer(self)
        self._timer_refresh_topics.timeout.connect(self.refresh_topics)

    def clicked_initialize_robot(self):
        if not self.__velma_command is None:
            self.__velma_command.initializeRobot()

    def clicked_enable_motors(self):
        if not self.__velma_command is None:
            self.__velma_command.enableMotors()

    def clicked_switch_to_jnt_imp(self):
        if not self.__velma_command is None:
            self.__velma_command.switchToJntImp()

    def clicked_switch_to_cart_imp(self):
        if not self.__velma_command is None:
            self.__velma_command.switchToCartImp()

    def clicked_switch_to_safe_col(self):
        if not self.__velma_command is None:
            self.__velma_command.switchToSafeCol()

    def clicked_move_to_initial_configuration(self):
        if not self.__velma_command is None:
            self.__velma_command.moveToInitialConfiguration()

    def clicked_clear_messages(self):
        #self.list_messages
        #self.button_clear_messages.
        pass

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
            if self.__velma_init.isInitialized():
                self.__velma = self.__velma_init.getVelmaInterface()
                self.__velma_command = VelmaCommandThread(self.__velma)
                self.__velma_command.start()
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
                msg = self.__velma_command.getNextMessage()
                if msg is None:
                    break
                item = QStandardItem(msg)
                self.__list_model.appendRow(item)
            self.list_messages.scrollToBottom()

    def shutdown_plugin(self):
        if not self.__velma_init is None:
            self.__velma_init.cancelAndWait()
        if not self.__velma_command is None:
            self.__velma_command.cancelAndWait()
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

