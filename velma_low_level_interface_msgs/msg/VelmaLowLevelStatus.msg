VelmaLowLevelStatusArm rArm
VelmaLowLevelStatusArm lArm

VelmaLowLevelStatusHand rHand
VelmaLowLevelStatusHand lHand

VelmaLowLevelStatusFT rFt
VelmaLowLevelStatusFT lFt

# torsoMotorPosition
float64 tMotor_q

## torsoMotorVelocity
float64 tMotor_dq

## headPanMotorPosition
float64 hpMotor_q

## headPanMotorVelocity
float64 hpMotor_dq

## headTiltMotorPosition
float64 htMotor_q

## headTiltMotorVelocity
float64 htMotor_dq

## RightHandTactile.BHPressureState_OUTPORT
barrett_hand_controller_msgs/BHPressureState rHand_p

## LeftHandOptoforce.force_0_OUTPORT
## LeftHandOptoforce.force_1_OUTPORT
## LeftHandOptoforce.force_2_OUTPORT
geometry_msgs/WrenchStamped[3] lHand_f

