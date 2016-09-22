uint32 test

VelmaLowLevelStatusArm rArm
bool rArm_valid
VelmaLowLevelStatusArm lArm
bool lArm_valid

VelmaLowLevelStatusHand rHand
bool rHand_valid
VelmaLowLevelStatusHand lHand
bool lHand_valid

VelmaLowLevelStatusFT rFt
bool rFt_valid
VelmaLowLevelStatusFT lFt
bool lFt_valid

VelmaLowLevelStatusMotor tMotor

# torsoMotorPosition
#float64 tMotor_q

## torsoMotorVelocity
#float64 tMotor_dq
#bool tMotor_valid

VelmaLowLevelStatusMotor hpMotor

## headPanMotorPosition
#float64 hpMotor_q

## headPanMotorVelocity
#float64 hpMotor_dq
#bool hpMotor_valid

VelmaLowLevelStatusMotor htMotor

## headTiltMotorPosition
#float64 htMotor_q

## headTiltMotorVelocity
#float64 htMotor_dq
#bool htMotor_valid

## RightHandTactile.BHPressureState_OUTPORT
barrett_hand_controller_msgs/BHPressureState rHand_p
bool tact_valid

## LeftHandOptoforce.force_0_OUTPORT
## LeftHandOptoforce.force_1_OUTPORT
## LeftHandOptoforce.force_2_OUTPORT
geometry_msgs/WrenchStamped[3] lHand_f
bool optoforce_valid

