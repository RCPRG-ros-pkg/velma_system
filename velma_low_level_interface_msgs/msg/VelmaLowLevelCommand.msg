float64[4] lHand_q
float64[4] rHand_q

float64[4] lHand_dq
float64[4] rHand_dq

float64[4] lHand_max_p
float64[4] rHand_max_p

float64[4] lHand_max_i
float64[4] rHand_max_i

bool lHand_hold
bool rHand_hold

int32 rHandTactile_cmd

# torsoMotorCurrentCommand
# headPanMotorCurrentCommand
# headTiltMotorCurrentCommand
float64 tMotor_i
float64 hpMotor_i
float64 htMotor_i

# headPanMotorPositionCommand
# headTiltMotorPositionCommand
float64 hpMotor_q
float64 htMotor_q

# headPanMotorVelocityCommand
# headTiltMotorVelocityCommand
float64 hpMotor_dq
float64 htMotor_dq

# leftArmJointTorqueCommand
# rightArmJointTorqueCommand
float64[7] lArm_t
float64[7] rArm_t

# leftKRLcmd
# rightKRLcmd
float64 robot_cmd

