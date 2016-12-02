uint32 test         # subsystem_buffer{type: port; data_type: uint32_t}

velma_core_ve_body_re_body_msgs/CommandSimple rTact     # subsystem_buffer{type: container; validity: rTact_valid}
bool rTact_valid

velma_core_ve_body_re_body_msgs/CommandMotor tMotor     # subsystem_buffer{type: container; validity: tMotor_valid}
bool tMotor_valid

velma_core_ve_body_re_body_msgs/CommandMotor hpMotor    # subsystem_buffer{type: container; validity: hpMotor_valid}
bool hpMotor_valid

velma_core_ve_body_re_body_msgs/CommandMotor htMotor    # subsystem_buffer{type: container; validity: htMotor_valid}
bool htMotor_valid

velma_core_ve_body_re_body_msgs/CommandArm lArm         # subsystem_buffer{type: container; validity: lArm_valid}
bool lArm_valid

velma_core_ve_body_re_body_msgs/CommandArm rArm         # subsystem_buffer{type: container; validity: rArm_valid}
bool rArm_valid

velma_core_ve_body_re_body_msgs/CommandHand lHand       # subsystem_buffer{type: container; validity: lHand_valid}
bool lHand_valid

velma_core_ve_body_re_body_msgs/CommandHand rHand       # subsystem_buffer{type: container; validity: rHand_valid}
bool rHand_valid

velma_core_ve_body_re_body_msgs/CommandSimple sc        # subsystem_buffer{type: container; validity: sc_valid}
bool sc_valid

