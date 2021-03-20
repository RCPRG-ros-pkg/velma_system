#!/usr/bin/env python
from __future__ import print_function
from __future__ import division

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import math
import tf

from velma_common import *
from rcprg_ros_utils import exitError

def move_jnt_imp(q_dest, time):
	global velma
	print("Moving to the required position...")
	velma.moveJoint(q_dest, time, start_time=0.5, position_tol=15.0/180.0*math.pi)
	error = velma.waitForJoint()
	if error != 0:
		print("The action should have ended without error, but the error code is", error)
		exitError(5)

def switch2cart_imp_mode(rORl, timeout):
	global velma
	print("Switch to cart_imp mode (no trajectory)...")
	if (rORl == "right"):
		if not velma.moveCartImpRightCurrentPos(start_time=0.5):
			exitError(7)
		if velma.waitForEffectorRight(timeout_s=timeout) != 0:
			exitError(8)
	elif (rORl == "left"):			
		if not velma.moveCartImpLeftCurrentPos(start_time=0.5):
			exitError(9)
		if velma.waitForEffectorLeft(timeout_s=timeout) != 0:
			exitError(10)	
	rospy.sleep(1.0)
	diag = velma.getCoreCsDiag()
	if not diag.inStateCartImp():
		print("The core_cs should be in cart_imp state, but it is not")
		exitError(11)

def switch2jnt_imp_mode():
	global velma
	print("Switch to jnt_imp mode (no trajectory)...")
	velma.moveJointImpToCurrentPos(start_time=0.5)
	error = velma.waitForJoint()
	if error != 0:
		print("The action should have ended without error, but the error code is", error)
		exitError(12)
	rospy.sleep(1.0)
	diag = velma.getCoreCsDiag()
	if not diag.inStateJntImp():
		print("The core_cs should be in jnt_imp state, but it is not")
		exitError(13)

def closeORopen_gripper(rORl, dest_q):
	global velma
	if (rORl == "right"):
		print("move of right gripper:", dest_q)
		velma.moveHandRight(dest_q, [1.5,1.5,1.5,1.5], [5000,5000,5000,5000], 2000, hold=True)
		if velma.waitForHandRight() != 0:
			exitError(14)
		rospy.sleep(0.1)
		if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q, tolerance=2.0):
			exitError(15)
	elif (rORl == "left"):
		print("move of left gripper:", dest_q)
		velma.moveHandLeft(dest_q, [1.5,1.5,1.5,1.5], [5000,5000,5000,5000], 2000, hold=True)
		if velma.waitForHandLeft() != 0:
			exitError(16)
		rospy.sleep(0.1)
		if not isHandConfigurationClose( velma.getHandLeftCurrentConfiguration(), dest_q, tolerance=2.0):
			exitError(17)

def tool2grip_pose(rORl):
	if (rORl == "right"):
		print("Moving the right tool and equilibrium pose from 'wrist' to 'grip' frame...")
		T_B_Wr = velma.getTf("B", "Wr")
		T_Wr_Gr = velma.getTf("Wr", "Gr")
		if not velma.moveCartImpRight([T_B_Wr*T_Wr_Gr], [0.5], [T_Wr_Gr], [0.5], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
			exitError(18)
		if velma.waitForEffectorRight() != 0:
			exitError(19)
		print("The right tool is now in 'grip' pose")
	elif (rORl == "left"):
		print("Moving the left tool and equilibrium pose from 'wrist' to 'grip' frame...")
		T_B_Wl = velma.getTf("B", "Wl")
		T_Wl_Gl = velma.getTf("Wl", "Gl")
		if not velma.moveCartImpLeft([T_B_Wl*T_Wl_Gl], [0.5], [T_Wl_Gl], [0.5], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
			exitError(20)
		if velma.waitForEffectorLeft() != 0:
			exitError(21)
		print("The left tool is now in 'grip' pose")			
	rospy.sleep(0.1)

def move_cart_imp(rORl, T_frame, imp_list, pt_val, time):
	t0 = 0.5
	max_wr = PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5))
	p_tol = PyKDL.Twist(PyKDL.Vector(pt_val,pt_val,pt_val), PyKDL.Vector(pt_val,pt_val,pt_val)) # path_tolerance
	if (rORl == "right"):
		print("Moving right wrist to pose defined in world frame...")
		if not velma.moveCartImpRight([T_frame], [time], None, None, [imp_list], [0.01], max_wrench = max_wr, start_time = t0, path_tol = p_tol):
			 exitError(22)
		if velma.waitForEffectorRight() != 0:
			 exitError(23)
	elif (rORl == "left"):
		print("Moving left wrist to pose defined in world frame...")		
		if not velma.moveCartImpLeft([T_frame], [time], None, None, [imp_list], [0.01], max_wrench = max_wr, start_time = t0, path_tol = p_tol):
			 exitError(24)
		if velma.waitForEffectorLeft() != 0:
			 exitError(25)			 
	rospy.sleep(0.1)

def identification_movements(rightORleft, imp_list, pt_val, dest_qGrasped, dest_qOpened, T_B_Trd, T_B_Trd_Identif1, T_B_Trd_Identif2):
	imp_list_id = PyKDL.Wrench(PyKDL.Vector(1000, 1000, 1000), PyKDL.Vector(400, 400, 400))
	# checking finger's configuration required to grasped the object
	print("*** start of identification procedure ***")
	move_cart_imp(rightORleft, T_B_Trd, imp_list_id, pt_val, 8.0)
	switch2jnt_imp_mode()
	closeORopen_gripper(rightORleft, dest_qGrasped)
	isHandConfigClose_rORl(rightORleft)
	q_hand = velma.getHandCurrentConfiguration(rightORleft)
	dest_qChecked = [q_hand[1], q_hand[4], q_hand[6], 0]
	print("[FingerOneKnuckleTwo, FingerTwoKnuckleTwo, FingerThreeKnuckleTwo, Spread]:", dest_qChecked)
	rospy.sleep(0.5)
	closeORopen_gripper(rightORleft, dest_qOpened)
	print("Velma's", rightORleft, "gripper opened")

	# taking mesurements before grasping the object
	switch2cart_imp_mode(rightORleft, 1.0)
	#exit(2)
	tool2grip_pose(rightORleft)	
	
	print("Moving gripper to first given pose and take first identification measurement")
	move_cart_imp(rightORleft, T_B_Trd_Identif1, imp_list_id, pt_val, 8.0)
	closeORopen_gripper(rightORleft, dest_qChecked)
	rospy.sleep(0.5)
	velma.sendIdentificationMeasurementCommand(rightORleft, 1)
	rospy.sleep(0.5)
	print("Moving gripper to second given pose and take second identification measurement")	
	move_cart_imp(rightORleft, T_B_Trd_Identif2, imp_list_id, pt_val, 8.0)
	rospy.sleep(0.5)
	velma.sendIdentificationMeasurementCommand(rightORleft, 2)
	rospy.sleep(0.5)
	move_cart_imp(rightORleft, T_B_Trd_Identif1, imp_list_id, pt_val, 8.0)

	# opening gripper
	switch2jnt_imp_mode()
	closeORopen_gripper(rightORleft, dest_qOpened)
	print("Velma's", rightORleft, "gripper opened")

	# grabbing the object	
	switch2cart_imp_mode(rightORleft, 1.0)
	tool2grip_pose(rightORleft)		
	print("Moving gripper to pose before grasping object...")	
	move_cart_imp(rightORleft, T_B_Trd, imp_list_id, pt_val, 8.0)	
	switch2jnt_imp_mode()
	closeORopen_gripper(rightORleft, dest_qGrasped)
	isHandConfigClose_rORl(rightORleft)

	# taking mesurements after grasping the object 
	switch2cart_imp_mode(rightORleft, 1.0)
	tool2grip_pose(rightORleft)	
	print("Moving gripper to first given pose and take third identification measurement")	
	move_cart_imp(rightORleft, T_B_Trd_Identif1, imp_list_id, pt_val, 8.0)
	rospy.sleep(0.5)
	velma.sendIdentificationMeasurementCommand(rightORleft, 3)
	rospy.sleep(0.5)
	print("Moving gripper to second given pose and take fourth identification measurement")	
	move_cart_imp(rightORleft, T_B_Trd_Identif2, imp_list_id, pt_val, 12.0)
	rospy.sleep(0.5)
	velma.sendIdentificationMeasurementCommand(rightORleft, 4)
	rospy.sleep(0.5)
	move_cart_imp(rightORleft, T_B_Trd_Identif1, imp_list, pt_val, 8.0)
	print("*** end of identification procedure ***")

def switch_weight_compensation(rightORleft, GRAV_OBJ_COMP, imp_list, pt_val):
	if(rightORleft == 'right'):
		T_B_Trd = velma.getTf("B", "Gr")		
	elif(rightORleft == 'left'):
		T_B_Trd = velma.getTf("B", "Gl")
	velma.setGraspedFlag(rightORleft, GRAV_OBJ_COMP)
	move_cart_imp(rightORleft, T_B_Trd, imp_list, pt_val, 2.0)

def diff_calc(rORl, T_frame):
	print("Calculating difference between desiread and reached pose...")
	if (rORl == "right"):	
		T_B_T_diff = PyKDL.diff(T_frame, velma.getTf("B", "Tr"), 1.0)
	elif (rORl == "left"):
		T_B_T_diff = PyKDL.diff(T_frame, velma.getTf("B", "Tl"), 1.0)		
	print(T_B_T_diff)

def get_tf_rORl(rORl):
	if (rORl == "right"):	
		T_Wo_grip = velma.getTf("Wo", "Gr") # right gripper
	elif (rORl == "left"):
		T_Wo_grip = velma.getTf("Wo", "Gl") # left gripper		
	return T_Wo_grip

def isHandConfigClose_rORl(rORl):
	if (rORl == "right"):	
		if isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_qGrasped, tolerance=0.02):
			print("Object not grabbed")	
			exitError(26)
		else:
			print("Right gripper grabbed the object")
	elif (rORl == "left"):
		if isHandConfigurationClose( velma.getHandLeftCurrentConfiguration(), dest_qGrasped, tolerance=0.02):
			print("Object not grabbed")	
			exitError(27)
		else:
			print("Left gripper grabbed the object")

def q_map_rORl0(rORl):
	if (rORl == "right"):	
		q_mapR = {'torso_0_joint':0,
			'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
			'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
			'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
			'right_arm_3_joint':2.0,   'left_arm_3_joint':-0.85,
			'right_arm_4_joint':0,      'left_arm_4_joint':0,
			'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
			'right_arm_6_joint':0,      'left_arm_6_joint':0 }
		return q_mapR
	elif (rORl == "left"):
		q_mapL = {'torso_0_joint':0,
			'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
			'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
			'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
			'right_arm_3_joint':0.85,   'left_arm_3_joint':-2.0,
			'right_arm_4_joint':0,      'left_arm_4_joint':0,
			'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
			'right_arm_6_joint':0,      'left_arm_6_joint':0 }
		return q_mapL	

def q_map_rORl(rORl, torso_angle):
	if (rORl == "right"):	
		q_mapR = {'torso_0_joint':torso_angle,
			'right_arm_0_joint':0.6,    'left_arm_0_joint':0.3,
			'right_arm_1_joint':-1.1,   'left_arm_1_joint':1.8,
			'right_arm_2_joint':1.5,    'left_arm_2_joint':-1.25,
			'right_arm_3_joint':1.8,    'left_arm_3_joint':-0.85,
			'right_arm_4_joint':1.2,    'left_arm_4_joint':0,
			'right_arm_5_joint':-1.05,  'left_arm_5_joint':0.5,
			'right_arm_6_joint':0,      'left_arm_6_joint':0 }
		return q_mapR
	elif (rORl == "left"):
		q_mapL = {'torso_0_joint':torso_angle,
			'right_arm_0_joint':-0.3,   'left_arm_0_joint':-0.6,
			'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.1,
			'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.5,
			'right_arm_3_joint':0.85,   'left_arm_3_joint':-1.8,
			'right_arm_4_joint':0,      'left_arm_4_joint':-1.2,
			'right_arm_5_joint':-0.5,   'left_arm_5_joint':1.05,
			'right_arm_6_joint':0,      'left_arm_6_joint':0 }
		return q_mapL			


if __name__ == "__main__":

	# start position
	q_map_starting = {'torso_0_joint':0,
		'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
		'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
		'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
		'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
		'right_arm_4_joint':0,      'left_arm_4_joint':0,
		'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
		'right_arm_6_joint':0,      'left_arm_6_joint':0 }

	rospy.init_node('object_control_test') 
	rospy.sleep(0.5)

	################ INITIAL SETTINGS ###############
	rightORleft = 'right'
	#rightORleft = 'left'
	###------------------------------------------###
	CART_TEST = True
	#CART_TEST = False
	###------------------------------------------###
	GRAV_OBJ_COMP = True
	#GRAV_OBJ_COMP = False
	#################################################

	print("Chosen gripper:", rightORleft)
	print("CART_TEST:", CART_TEST)
	print("GRAV_OBJ_COMP:", GRAV_OBJ_COMP)

	# defining the parametres
	imp_list = PyKDL.Wrench(PyKDL.Vector(500, 500, 500), PyKDL.Vector(100, 100, 100)) # forces and torques
	pt_val = 0.5

	dest_qOpened = [0, 0, 0, 0]
	dest_qOpened2 = [0, 0, 0, math.pi]	
	dest_qClosed = [90.0/180.0*math.pi, 90.0/180.0*math.pi, 90.0/180.0*math.pi, 0]
	dest_qGrasped = [85.0/180.0*math.pi, 85.0/180.0*math.pi, 85.0/180.0*math.pi, 0]

	# initializing
	print("Running python interface for Velma...")
	velma = VelmaInterface()
	print("Waiting for VelmaInterface initialization...")
	if not velma.waitForInit(timeout_s=5.0):
		 print("Could not initialize VelmaInterface")
		 exitError(1)
	print("Initialization ok!")

	# rospy.sleep(10)
	# T_B_Gr = velma.getTf("B", "Gr")
	# print(T_B_Gr.p)
	# exit(0)

	diag = velma.getCoreCsDiag()
	if not diag.motorsReady():
		 print("Motors must be homed and ready to use for this test.")
		 exitError(2)

	if velma.enableMotors() != 0:
		 exitError(4)

	# nothing grasped
	velma.setGraspedFlag('right', False)
	velma.setGraspedFlag('left', False)

	# closing grippers
	switch2jnt_imp_mode()
	closeORopen_gripper("right", dest_qClosed)
	print("Right gripper closed")
	closeORopen_gripper("left", dest_qClosed)
	print("Left gripper closed")	

	#move_jnt_imp(q_map_starting, 15.0)
	#exit(1)

	# moving to proper configuration & opening gripper
	alfa = 0.0
	q_map_0 = q_map_rORl0(rightORleft)
	q_map_1 = q_map_rORl(rightORleft, alfa)
	move_jnt_imp(q_map_0, 4.0)
	move_jnt_imp(q_map_1, 8.0)
	closeORopen_gripper(rightORleft, dest_qOpened)
	print("Velma's", rightORleft, "gripper opened")
	#exit(101)

	# cart_imp_mode initializing
	switch2cart_imp_mode(rightORleft, 1.0)
	print("Reset tools for both arms...")
	T_B_Wr = velma.getTf("B", "Wr")
	T_B_Wl = velma.getTf("B", "Wl")
	if not velma.moveCartImpRight([T_B_Wr], [0.5], [PyKDL.Frame()], [0.5], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
		 exitError(29)
	if not velma.moveCartImpLeft([T_B_Wl], [0.5], [PyKDL.Frame()], [0.5], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
		 exitError(30)
	if velma.waitForEffectorRight() != 0:
		 exitError(31)
	if velma.waitForEffectorLeft() != 0:
		 exitError(32)

	# preparing to move gripper to the object in cart_imp_mode
	tool2grip_pose(rightORleft)
	T_Wo_grip = get_tf_rORl(rightORleft)
	# do odczytania object_frame --------------------------------------------------
	# T_Wo_obj1_pos = PyKDL.Vector(0.621926, -0.260334, 0.851752)
	if(rightORleft == 'right'):
		T_Wo_obj1_pos = PyKDL.Vector(0.8, -0.17, 1.1) # object	
	elif(rightORleft == 'left'):
		T_Wo_obj1_pos = PyKDL.Vector(0.8, 0.17, 1.1) # object
	
	T_B_Trd = PyKDL.Frame(T_Wo_grip.M, T_Wo_obj1_pos + PyKDL.Vector(0.0, 0.0, 0.0))

	dr =  (T_Wo_obj1_pos - T_Wo_grip.p)
	drL = math.sqrt(dr[0]**2 + dr[1]**2)
	T_B_Trd_Identif1 = PyKDL.Frame(T_Wo_grip.M, T_Wo_obj1_pos + PyKDL.Vector(-0.15*dr[0]/drL, -0.15*dr[1]/drL, 0.15))
	if (rightORleft == "right"):
		R_z = PyKDL.Rotation.RotZ(-math.pi/2)
	elif (rightORleft == "left"):
		R_z = PyKDL.Rotation.RotZ(math.pi/2)	
	T_B_Trd_Identif2 = PyKDL.Frame(T_Wo_grip.M*R_z, T_Wo_obj1_pos + PyKDL.Vector(-0.15*dr[0]/drL, -0.15*dr[1]/drL, 0.15))

	# identification procedure
	identification_movements(rightORleft, imp_list, pt_val, dest_qGrasped, dest_qOpened, T_B_Trd, T_B_Trd_Identif1, T_B_Trd_Identif2)

	# use identified params or not
	switch_weight_compensation(rightORleft, GRAV_OBJ_COMP, imp_list, pt_val)

	# moving gripper up
	print("Moving gripper up...")	
	T_B_Trd = PyKDL.Frame(T_Wo_grip.M, T_Wo_obj1_pos + PyKDL.Vector(0, 0, 0.4))
	move_cart_imp(rightORleft, T_B_Trd, imp_list, pt_val, 3.0)

	if(CART_TEST == True):
		q_map_2 = q_map_rORl(rightORleft, 0.0)
		switch2jnt_imp_mode()
		move_jnt_imp(q_map_2, 4.0)	

		if(rightORleft == 'right'):
			pm = 1.0
			quat_z = 0.0
			quat_w = 1.0
		elif(rightORleft == 'left'):
			pm = -1.0
			quat_z = math.sin(math.pi/2) # 1.0 -> sin(theta/2)
			quat_w = math.cos(math.pi/2) # 0.0 -> cos(theta/2)
		else:
			print("rightORleft and pm problem...")
			exitError(34)

		T_B_Trd2 = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , quat_z , quat_w ), PyKDL.Vector( 0.7, -0.2*pm, 1.45))
		T_B_Trd3 = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , quat_z , quat_w ), PyKDL.Vector( 0.7, -0.2*pm, 1.35))
		T_B_Trd4 = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , quat_z , quat_w ), PyKDL.Vector( 0.7, -0.1*pm, 1.35))		
		T_B_Trd5 = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , quat_z , quat_w ), PyKDL.Vector( 0.7, -0.1*pm, 1.45))

		switch2cart_imp_mode(rightORleft, 1.0) # optional
		move_cart_imp(rightORleft, T_B_Trd2, imp_list, pt_val, 3.0)
		diff_calc(rightORleft, T_B_Trd2)

		i = 0
		while i < 2:
			move_cart_imp(rightORleft, T_B_Trd3, imp_list, pt_val, 3.0)
			diff_calc(rightORleft, T_B_Trd3)			
			move_cart_imp(rightORleft, T_B_Trd4, imp_list, pt_val, 3.0)
			diff_calc(rightORleft, T_B_Trd4)
			move_cart_imp(rightORleft, T_B_Trd5, imp_list, pt_val, 3.0)
			diff_calc(rightORleft, T_B_Trd5)
			move_cart_imp(rightORleft, T_B_Trd2, imp_list, pt_val, 3.0)
			diff_calc(rightORleft, T_B_Trd2)
			rospy.sleep(1.0)
			i += 1
	
	# calculating the angle and moving to proper configuration
	switch2jnt_imp_mode()
	q_map_end = q_map_rORl(rightORleft, alfa)
	move_jnt_imp(q_map_end, 8.0)

	# procedure of putting down the object 
	switch2cart_imp_mode(rightORleft, 1.0)
	tool2grip_pose(rightORleft)
	T_Wo_grip = get_tf_rORl(rightORleft)		

	print("Moving gripper to object's destination")	
	# do wpisania --------------------------------------------------------
	T_B_Trd = PyKDL.Frame(T_Wo_grip.M, T_Wo_obj1_pos + PyKDL.Vector(0.0, 0.0, 0.01))
	move_cart_imp(rightORleft, T_B_Trd, imp_list, pt_val, 12.0)
	#switch_weight_compensation(rightORleft, False, imp_list, pt_val)	

	# opening gripper
	switch2jnt_imp_mode()
	[time, q_actual] = velma.getLastJointState()
	velma.setGraspedFlag(rightORleft, False)	
	move_jnt_imp(q_actual, 2.0)	
	closeORopen_gripper(rightORleft, dest_qOpened)
	print("Velma's", rightORleft, "gripper opened")

	# moving gripper up
	switch2cart_imp_mode(rightORleft, 1.0)
	tool2grip_pose(rightORleft)	
	print("Moving", rightORleft, "gripper up...")

	# do wpisania --------------------------------------------------------
	T_B_Trd = PyKDL.Frame(T_Wo_grip.M, T_Wo_obj1_pos + PyKDL.Vector(0.0, 0.0, 0.4))
	move_cart_imp(rightORleft, T_B_Trd, imp_list, pt_val, 8.0)

	# closing gripper
	switch2jnt_imp_mode()
	closeORopen_gripper(rightORleft, dest_qClosed)
	print("Velma's", rightORleft, "gripper closed")

	# moving to start position
	move_jnt_imp(q_map_1, 4.0)
	move_jnt_imp(q_map_0, 8.0)	
	move_jnt_imp(q_map_starting, 4.0)

	# opening grippers
	# closeORopen_gripper("right", dest_qOpened)
	# print("Right gripper opened")
	# closeORopen_gripper("left", dest_qOpened)
	# print("Left gripper opened")

	rospy.sleep(0.5)

	exitError(0)
