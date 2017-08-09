StatusCartImp rCart
bool rCart_valid

StatusCartImp lCart
bool lCart_valid

StatusJntImp jnt
bool jnt_valid

StatusHead head
bool head_valid

float64[33] q
bool q_valid

float64[33] dq
bool dq_valid

geometry_msgs/Pose rWristPose
bool rWristPose_valid

geometry_msgs/Pose lWristPose
bool lWristPose_valid

geometry_msgs/Pose rToolInWristPose
bool rToolInWristPose_valid

geometry_msgs/Pose lToolInWristPose
bool lToolInWristPose_valid

barrett_hand_msgs/StatusHand handRight
bool handRight_valid

barrett_hand_msgs/StatusHand handLeft
bool handLeft_valid

barrett_hand_status_msgs/BHPressureState p
bool p_valid

geometry_msgs/Wrench[3] f
bool f_valid

StatusServo hp

StatusServo ht

StatusServo t

#bool homing_required_hp
#bool homing_required_hp_valid

#bool homing_in_progress_hp
#bool homing_in_progress_hp_valid

#bool homing_required_ht
#bool homing_required_ht_valid

#bool homing_in_progress_ht
#bool homing_in_progress_ht_valid

uint32 STATE_IDLE = 0
uint32 STATE_SAFE = 1
uint32 STATE_CART_IMP = 2
uint32 STATE_JNT_IMP = 3
uint32 STATE_FCL = 3

uint32 subsystem_state
bool subsystem_state_valid

