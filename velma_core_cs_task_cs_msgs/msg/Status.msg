StatusCartImp rCart     # subsystem_buffer{type: container; validity: rCart_valid}
bool rCart_valid

StatusCartImp lCart     # subsystem_buffer{type: container; validity: rCart_valid}
bool lCart_valid

StatusJntImp jnt        # subsystem_buffer{type: container; validity: jnt_valid}
bool jnt_valid

float64[33] q           # subsystem_buffer{type: port; validity: q_valid}
bool q_valid

float64[33] dq          # subsystem_buffer{type: port; validity: dq_valid}
bool dq_valid

geometry_msgs/Pose rWristPose   # subsystem_buffer{type: port; validity: rWristPose_valid}
bool rWristPose_valid

geometry_msgs/Pose lWristPose   # subsystem_buffer{type: port; validity: lWristPose_valid}
bool lWristPose_valid

geometry_msgs/Pose rToolInWristPose                 # subsystem_buffer{type: port; validity: rToolInWristPose_valid}
bool rToolInWristPose_valid

geometry_msgs/Pose lToolInWristPose                 # subsystem_buffer{type: port; validity: lToolInWristPose_valid}
bool lToolInWristPose_valid

velma_core_cs_ve_body_msgs/StatusHand hand_r   # subsystem_buffer{type: container; validity: hand_r_valid}
bool hand_r_valid

velma_core_cs_ve_body_msgs/StatusHand hand_l   # subsystem_buffer{type: container; validity: hand_l_valid}
bool hand_l_valid

barrett_hand_controller_msgs/BHPressureState p      # subsystem_buffer{type: port; validity: p_valid}
bool p_valid

geometry_msgs/Wrench[3] f                           # subsystem_buffer{type: port; validity: f_valid}
bool f_valid

uint32 STATE_IDLE = 0
uint32 STATE_SAFE = 1
uint32 STATE_CART_IMP = 2
uint32 STATE_JNT_IMP = 3
uint32 STATE_FCL = 3

uint32 subsystem_state                             # subsystem_buffer{type: port; validity: subsystem_state_valid}
bool subsystem_state_valid

