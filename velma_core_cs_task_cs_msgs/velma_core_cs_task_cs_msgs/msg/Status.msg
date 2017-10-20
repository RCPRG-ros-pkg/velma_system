StatusCartImp rCart     # status of left cartesian impedance trajectory generators
bool rCart_valid

StatusCartImp lCart     # status of left cartesian impedance trajectory generators
bool lCart_valid

StatusJntImp jnt        # status of joint impedance trajectory generator
bool jnt_valid

StatusHead head         # status of head trajectory generator
bool head_valid

float64[33] q       # position of all joints
bool q_valid

float64[33] dq      # velocity of all joints
bool dq_valid

geometry_msgs/Pose rWristPose           # pose of right wrist wrt. base
bool rWristPose_valid

geometry_msgs/Pose lWristPose           # pose of left wrist wrt. base
bool lWristPose_valid

geometry_msgs/Pose rToolInWristPose     # pose of right tool wrt. right wrist
bool rToolInWristPose_valid

geometry_msgs/Pose lToolInWristPose     # pose of left tool wrt. left wrist
bool lToolInWristPose_valid

barrett_hand_msgs/StatusHand handRight  # status of right hand
bool handRight_valid

barrett_hand_msgs/StatusHand handLeft   # status of left hand
bool handLeft_valid

barrett_hand_status_msgs/BHPressureState p      # state of tactile sensors
bool p_valid

geometry_msgs/Wrench[3] f                       # state of optoforce sensors
bool f_valid

StatusServo hp      # status of head pan motor

StatusServo ht      # status of head tilt motor

StatusServo t       # status of torso motor

# states of control subsystem
uint32 STATE_IDLE = 0
uint32 STATE_SAFE = 1
uint32 STATE_CART_IMP = 2
uint32 STATE_JNT_IMP = 3
uint32 STATE_SAFE_COL = 4
uint32 STATE_FCL = 5   # currently unused

uint32 subsystem_state
bool subsystem_state_valid

