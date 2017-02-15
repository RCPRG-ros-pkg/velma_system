#StatusCartImp cart      # subsystem_buffer{type: port; data_type: StatusCartImp; validity: cart_valid}
#bool cart_valid

StatusJntImp jnt        # subsystem_buffer{type: container; validity: jnt_valid}
bool jnt_valid

float64[33] q           # subsystem_buffer{type: port; validity: q_valid}
bool q_valid

float64[33] dq          # subsystem_buffer{type: port; validity: dq_valid}
bool dq_valid

#float64[33] t

#geometry_msgs/Pose 

# TODO

