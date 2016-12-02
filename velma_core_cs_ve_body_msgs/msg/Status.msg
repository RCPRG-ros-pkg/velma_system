uint32 test         # subsystem_buffer{type: port; data_type: uint32_t}

StatusSC sc         # subsystem_buffer{type: port; data_type: StatusSC; validity: sc_valid}
bool sc_valid

velma_core_ve_body_re_body_msgs/StatusArm rArm          # subsystem_buffer{type: container; validity: rArm_valid}
bool rArm_valid

velma_core_ve_body_re_body_msgs/StatusArm lArm          # subsystem_buffer{type: container; validity: lArm_valid}
bool lArm_valid

velma_core_ve_body_re_body_msgs/StatusHand rHand        # subsystem_buffer{type: container; validity: rHand_valid}
bool rHand_valid

velma_core_ve_body_re_body_msgs/StatusHand lHand        # subsystem_buffer{type: container; validity: lHand_valid}
bool lHand_valid

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

barrett_hand_controller_msgs/BHPressureState rHand_p        # subsystem_buffer{type: port; data_type: barrett_hand_controller_msgs::BHPressureState; validity: rHand_p_valid}
bool rHand_p_valid

geometry_msgs/Wrench[3] lHand_f                             # subsystem_buffer{type: port; data_type: boost::array< ::geometry_msgs::Wrench, 3>; validity: lHand_f_valid}
bool lHand_f_valid

