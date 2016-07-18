# leftHandStatus
# rightHandStatus
int32 lHand_s
int32 rHand_s

# leftHandJointPosition
# rightHandJointPosition
float64 lHand_q
float64 rHand_q

# RightHandTactile.BHPressureState_OUTPORT
float

LeftHandOptoforce.force_0_OUTPORT
LeftHandOptoforce.force_1_OUTPORT
LeftHandOptoforce.force_2_OUTPORT

# torsoMotorPosition
float64 tMotor_q

# torsoMotorVelocity
float64 tMotor_dq

# headPanMotorPosition
float64 hpMotor_q

# headPanMotorVelocity
float64 hpMotor_dq

# headTiltMotorPosition
float64 htMotor_q

# headTiltMotorVelocity
float64 htMotor_dq

# leftArmJointPosition
# rightArmJointPosition
float64[7] lArm_q
float64[7] rArm_q

# leftArmJointVelocity
# rightArmJointVelocity
float64[7] lArm_dq
float64[7] rArm_dq

# leftArmJointTorque
# rightArmJointTorque
float64[7] lArm_t
float64[7] rArm_t

# leftArmWrench
# rightArmWrench
float64[6] lArm_w
float64[6] rArm_w

# leftArmMassMatrix
# rightArmMassMatrix
float64[28] lArm_mm
float64[28] rArm_mm

# leftArmGravityTorque
# rightArmGravityTorque
float64[7] lArm_gt
float64[7] rArm_gt

# leftFtSensorRawWrench
# rightFtSensorRawWrench
float64[6] lFTSensor_rw
float64[6] rFTSensor_rw

# leftFtSensorFastFilteredWrench
# rightFtSensorFastFilteredWrench
float64[6] lFTSensor_ffw
float64[6] rFTSensor_ffw

# leftFtSensorSlowFilteredWrench
# rightFtSensorSlowFilteredWrench
float64[6] lFTSensor_sfw
float64[6] rFTSensor_sfw

# leftArmRobotState_OUTPORT_name()           { return string("FRIl.RobotState_OUTPORT"); }
# leftArmFRIState_OUTPORT_name()             { return string("FRIl.FRIState_OUTPORT"); }
# rightArmRobotState_OUTPORT_name()          { return string("FRIr.RobotState_OUTPORT"); }
# rightArmFRIState_OUTPORT_name()            { return string("FRIr.FRIState_OUTPORT"); }
int32 robot_s

