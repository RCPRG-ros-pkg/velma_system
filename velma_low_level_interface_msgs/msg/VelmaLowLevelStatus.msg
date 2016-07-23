VelmaLowLevelStatusHand rHand
VelmaLowLevelStatusHand lHand

VelmaLowLevelStatusArm rArm
VelmaLowLevelStatusArm lArm


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


