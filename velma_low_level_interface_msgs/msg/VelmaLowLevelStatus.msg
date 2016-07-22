## leftHandStatus
## rightHandStatus
#int32 lHand_s
#int32 rHand_s

## leftHandJointPosition
## rightHandJointPosition
#float64 lHand_q
#float64 rHand_q

## RightHandTactile.BHPressureState_OUTPORT
#barrett_hand_controller_msgs/BHPressureState rHand_p

## LeftHandOptoforce.force_0_OUTPORT
## LeftHandOptoforce.force_1_OUTPORT
## LeftHandOptoforce.force_2_OUTPORT
#geometry_msgs/WrenchStamped[3] lHand_f

# torsoMotorPosition
float64 tMotor_q

## torsoMotorVelocity
#float64 tMotor_dq

## headPanMotorPosition
#float64 hpMotor_q

## headPanMotorVelocity
#float64 hpMotor_dq

## headTiltMotorPosition
#float64 htMotor_q

## headTiltMotorVelocity
#float64 htMotor_dq

## leftArmJointPosition
## rightArmJointPosition
#float64[7] lArm_q
#float64[7] rArm_q

## leftArmJointVelocity
## rightArmJointVelocity
#float64[7] lArm_dq
#float64[7] rArm_dq

## leftArmJointTorque
## rightArmJointTorque
#float64[7] lArm_t
#float64[7] rArm_t

## leftArmWrench
## rightArmWrench
#geometry_msgs/Wrench lArm_w
#geometry_msgs/Wrench rArm_w

## leftArmMassMatrix
## rightArmMassMatrix
#float64[28] lArm_mm
#float64[28] rArm_mm

## leftArmGravityTorque
## rightArmGravityTorque
#float64[7] lArm_gt
#float64[7] rArm_gt

## leftFtSensorRawWrench
## rightFtSensorRawWrench
#geometry_msgs/Wrench lFTSensor_rw
#geometry_msgs/Wrench rFTSensor_rw

## leftFtSensorFastFilteredWrench
## rightFtSensorFastFilteredWrench
#geometry_msgs/Wrench lFTSensor_ffw
#geometry_msgs/Wrench rFTSensor_ffw

## leftFtSensorSlowFilteredWrench
## rightFtSensorSlowFilteredWrench
#geometry_msgs/Wrench lFTSensor_sfw
#geometry_msgs/Wrench rFTSensor_sfw

#uint8[40] rArm_tFriIntfState
#uint8[36] rArm_tFriRobotState

#uint8[40] lArm_tFriIntfState
#uint8[36] lArm_tFriRobotState

