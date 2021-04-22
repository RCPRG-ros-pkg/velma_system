# This field is used to check if all messages are consumed
uint32 test
bool test_valid

# Statu for error diagnostics
StatusSC sc
bool sc_valid

# Status of the right arm
StatusArm rArm
bool rArm_valid

# Status of the left arm
StatusArm lArm
bool lArm_valid

# Status of the right force/torque sensor
StatusFT rFt
bool rFt_valid

# Status of the left force/torque sensor
StatusFT lFt
bool lFt_valid

# Status of torso motor
StatusMotor tMotor
bool tMotor_valid

# Status of head pan motor
StatusMotor hpMotor
bool hpMotor_valid

# Status of head tilt motor
StatusMotor htMotor
bool htMotor_valid

# Status of the right hand
barrett_hand_msgs/StatusHand rHand
bool rHand_valid

# Status of the left hand
barrett_hand_msgs/StatusHand lHand
bool lHand_valid

# Status of tactile sensors
barrett_hand_status_msgs/BHPressureState p
bool p_valid

# Status of Optoforce sensors
geometry_msgs/Wrench[3] f
bool f_valid
