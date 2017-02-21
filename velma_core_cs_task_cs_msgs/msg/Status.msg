StatusCartImp rCart      # subsystem_buffer{type: container; validity: rCart_valid}
bool rCart_valid

StatusCartImp lCart      # subsystem_buffer{type: container; validity: rCart_valid}
bool lCart_valid

StatusJntImp jnt        # subsystem_buffer{type: container; validity: jnt_valid}
bool jnt_valid

float64[33] q           # subsystem_buffer{type: port; validity: q_valid}
bool q_valid

float64[33] dq          # subsystem_buffer{type: port; validity: dq_valid}
bool dq_valid

geometry_msgs/Pose rWristPose    # subsystem_buffer{type: port; validity: rWristPose_valid}
bool rWristPose_valid

geometry_msgs/Pose lWristPose    # subsystem_buffer{type: port; validity: lWristPose_valid}
bool lWristPose_valid

