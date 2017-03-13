uint32 test         # subsystem_buffer{type: port; data_type: uint32_t}

StatusSC sc         # subsystem_buffer{type: port; data_type: StatusSC; validity: sc_valid}
bool sc_valid

velma_core_ve_body_re_body_msgs/StatusArm rArm          # subsystem_buffer{type: container; validity: rArm_valid}
bool rArm_valid

velma_core_ve_body_re_body_msgs/StatusArm lArm          # subsystem_buffer{type: container; validity: lArm_valid}
bool lArm_valid

velma_core_ve_body_re_body_msgs/StatusFT rFt            # subsystem_buffer{type: container; validity: rFt_valid}
bool rFt_valid

velma_core_ve_body_re_body_msgs/StatusFT lFt            # subsystem_buffer{type: container; validity: lFt_valid}
bool lFt_valid

velma_core_ve_body_re_body_msgs/StatusMotor tMotor      # subsystem_buffer{type: container; validity: tMotor_valid}
bool tMotor_valid

velma_core_ve_body_re_body_msgs/StatusMotor hpMotor     # subsystem_buffer{type: container; validity: hpMotor_valid}
bool hpMotor_valid

velma_core_ve_body_re_body_msgs/StatusMotor htMotor     # subsystem_buffer{type: container; validity: htMotor_valid}
bool htMotor_valid

