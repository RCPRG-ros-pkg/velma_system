CommandCartImp cart_r   # subsystem_buffer{type: container}

CommandCartImp cart_l   # subsystem_buffer{type: container}

CommandJntImp jnt       # subsystem_buffer{type: port; validity: jnt_valid}
bool jnt_valid

CommandHead head        # subsystem_buffer{type: port; validity: head_valid}
bool head_valid

velma_core_cs_ve_body_msgs/CommandHand hand_r   # subsystem_buffer{type: container; validity: hand_r_valid}
bool hand_r_valid

velma_core_cs_ve_body_msgs/CommandHand hand_l   # subsystem_buffer{type: container; validity: hand_l_valid}
bool hand_l_valid

