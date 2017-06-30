CommandCartImp cart_r   # subsystem_buffer{type: container}

CommandCartImp cart_l   # subsystem_buffer{type: container}

CommandJntImp jnt       # subsystem_buffer{type: port; validity: jnt_valid}
bool jnt_valid

CommandHead head        # subsystem_buffer{type: port; validity: head_valid}
bool head_valid

barrett_hand_msgs/CommandHand handRight    # subsystem_buffer{type: container; validity: handRight_valid}
bool handRight_valid

bool handRightReset     # subsystem_buffer{type: port; validity: handRightReset_valid}
bool handRightReset_valid

barrett_hand_msgs/CommandHand handLeft     # subsystem_buffer{type: container; validity: handLeft_valid}
bool handLeft_valid

bool handLeftReset      # subsystem_buffer{type: port; validity: handLeftReset_valid}
bool handLeftReset_valid

