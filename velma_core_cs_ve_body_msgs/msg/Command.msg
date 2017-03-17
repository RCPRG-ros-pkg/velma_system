uint32 test             # subsystem_buffer{type: port; data_type: uint32_t}

float64 tMotor_i        # subsystem_buffer{type: port; validity: tMotor_i_valid}
bool tMotor_i_valid

velma_core_ve_body_re_body_msgs/CommandArm lArm         # subsystem_buffer{type: container; validity: lArm_valid}
bool lArm_valid

velma_core_ve_body_re_body_msgs/CommandArm rArm         # subsystem_buffer{type: container; validity: rArm_valid}
bool rArm_valid

int32 sc                # subsystem_buffer{type: port; validity: sc_valid}
bool sc_valid

int32 tact              # subsystem_buffer{type: port; validity: tact_valid}
bool tact_valid

velma_core_ve_body_re_body_msgs/CommandHand lHand       # subsystem_buffer{type: container; validity: lHand_valid}
bool lHand_valid

velma_core_ve_body_re_body_msgs/CommandHand rHand       # subsystem_buffer{type: container; validity: rHand_valid}
bool rHand_valid

