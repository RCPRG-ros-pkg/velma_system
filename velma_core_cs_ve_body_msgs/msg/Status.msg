uint32 test             # subsystem_buffer{type: port; data_type: uint32_t}

StatusSC sc             # subsystem_buffer{type: port; data_type: StatusSC; validity: sc_valid}
bool sc_valid

StatusArm rArm          # subsystem_buffer{type: container; validity: rArm_valid}
bool rArm_valid

StatusArm lArm          # subsystem_buffer{type: container; validity: lArm_valid}
bool lArm_valid

StatusFT rFt            # subsystem_buffer{type: container; validity: rFt_valid}
bool rFt_valid

StatusFT lFt            # subsystem_buffer{type: container; validity: lFt_valid}
bool lFt_valid

StatusMotor tMotor      # subsystem_buffer{type: container; validity: tMotor_valid}
bool tMotor_valid

StatusMotor hpMotor     # subsystem_buffer{type: container; validity: hpMotor_valid}
bool hpMotor_valid

StatusMotor htMotor     # subsystem_buffer{type: container; validity: htMotor_valid}
bool htMotor_valid

StatusHand rHand        # subsystem_buffer{type: container; validity: rHand_valid}
bool rHand_valid

StatusHand lHand        # subsystem_buffer{type: container; validity: lHand_valid}
bool lHand_valid

barrett_hand_controller_msgs/BHPressureState p  # subsystem_buffer{type: port; validity: p_valid}
bool p_valid

geometry_msgs/Wrench[3] f                       # subsystem_buffer{type: port; validity: f_valid}
bool f_valid
