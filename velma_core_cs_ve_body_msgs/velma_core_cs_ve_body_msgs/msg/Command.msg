# This field is used to check if all messages are consumed
uint32 test
bool test_valid

# Torso motor torque
float64 tMotor_t
bool tMotor_t_valid

# Enable flag for torso motor
bool tMotor_enable
bool tMotor_enable_valid

# Command for left arm
CommandArm lArm
bool lArm_valid

# Command for right arm
CommandArm rArm
bool rArm_valid

# Command for head pan motor
CommandMotor hpMotor
bool hpMotor_valid

# Command for head tilt motor
CommandMotor htMotor
bool htMotor_valid

# Command to exit safe state (when sc == 1)
int32 sc
bool sc_valid

# Currently not used
#int32 tact
#bool tact_valid

# Command for left hand - move fingers
barrett_hand_msgs/CommandHand lHand
bool lHand_valid

# Command for left hand - reset
bool lHandReset
bool lHandReset_valid

# Command for right hand - move fingers
barrett_hand_msgs/CommandHand rHand
bool rHand_valid

# Command for right hand - reset
bool rHandReset
bool rHandReset_valid
