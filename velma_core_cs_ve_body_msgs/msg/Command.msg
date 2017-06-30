uint32 test             # subsystem_buffer{type: port; data_type: uint32_t}

float64 tMotor_i        # subsystem_buffer{type: port; validity: tMotor_i_valid}
bool tMotor_i_valid

CommandArm lArm         # subsystem_buffer{type: container; validity: lArm_valid}
bool lArm_valid

CommandArm rArm         # subsystem_buffer{type: container; validity: rArm_valid}
bool rArm_valid

CommandMotor hpMotor    # subsystem_buffer{type: container; validity: hpMotor_valid}
bool hpMotor_valid

CommandMotor htMotor    # subsystem_buffer{type: container; validity: htMotor_valid}
bool htMotor_valid

int32 sc                # subsystem_buffer{type: port; validity: sc_valid}
bool sc_valid

int32 tact              # subsystem_buffer{type: port; validity: tact_valid}
bool tact_valid

barrett_hand_msgs/CommandHand lHand       # subsystem_buffer{type: container; validity: lHand_valid}
bool lHand_valid

bool lHandReset         # subsystem_buffer{type: port; validity: lHandReset_valid}
bool lHandReset_valid

barrett_hand_msgs/CommandHand rHand       # subsystem_buffer{type: container; validity: rHand_valid}
bool rHand_valid

bool rHandReset         # subsystem_buffer{type: port; validity: rHandReset_valid}
bool rHandReset_valid

