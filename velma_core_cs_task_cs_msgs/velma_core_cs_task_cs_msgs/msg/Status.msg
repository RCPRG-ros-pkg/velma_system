# Status of right cartesian impedance trajectory generators
StatusCartImp rCart
bool rCart_valid

# Status of left cartesian impedance trajectory generators
StatusCartImp lCart
bool lCart_valid

# Status of joint impedance trajectory generator
StatusJntImp jnt
bool jnt_valid

# Status of head trajectory generator
StatusHead head
bool head_valid

# Position of all joints
float64[33] q
bool q_valid

# Velocity of all joints
float64[33] dq
bool dq_valid

# Pose of the right wrist wrt. base
geometry_msgs/Pose rWristPose
bool rWristPose_valid

# Pose of left wrist wrt. base
geometry_msgs/Pose lWristPose
bool lWristPose_valid

# Pose of right tool wrt. right wrist
geometry_msgs/Pose rToolInWristPose
bool rToolInWristPose_valid

# Pose of left tool wrt. left wrist
geometry_msgs/Pose lToolInWristPose
bool lToolInWristPose_valid

# Pose of right tool wrt. base
geometry_msgs/Pose rToolInBasePose
bool rToolInBasePose_valid

# Pose of left tool wrt. base
geometry_msgs/Pose lToolInBasePose
bool lToolInBasePose_valid

# Status of the right hand
barrett_hand_msgs/StatusHand handRight
bool handRight_valid

# Status of the left hand
barrett_hand_msgs/StatusHand handLeft
bool handLeft_valid

# State of tactile sensors
barrett_hand_status_msgs/BHPressureState p
bool p_valid

# State of optoforce sensors
geometry_msgs/Wrench[3] f
bool f_valid

# Status of head pan motor
StatusServo hp

# Status of head tilt motor
StatusServo ht

# Status of torso motor
StatusServo t

# States of control subsystem
uint32 STATE_IDLE = 0
uint32 STATE_SAFE = 1
uint32 STATE_CART_IMP = 2
uint32 STATE_JNT_IMP = 3
uint32 STATE_SAFE_COL = 4
uint32 STATE_FCL = 5   # currently unused

# Current state of control subsystem
uint32 subsystem_state
bool subsystem_state_valid

# Total commanded torque for the right arm
float64[7] rArm_t

# Total commanded torque for the left arm
float64[7] lArm_t

# Total commanded torque for the torso
float64 torso_t
