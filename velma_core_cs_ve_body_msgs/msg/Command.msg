uint32 test         # subsystem_buffer{type: port; data_type: uint32_t}

velma_core_ve_body_re_body_msgs/CommandMotor tMotor     # subsystem_buffer{type: container; validity: tMotor_valid}
bool tMotor_valid

velma_core_ve_body_re_body_msgs/CommandArm lArm         # subsystem_buffer{type: container; validity: lArm_valid}
bool lArm_valid

velma_core_ve_body_re_body_msgs/CommandArm rArm         # subsystem_buffer{type: container; validity: rArm_valid}
bool rArm_valid

velma_core_ve_body_re_body_msgs/CommandSimple sc        # subsystem_buffer{type: container; validity: sc_valid}
bool sc_valid

