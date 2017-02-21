geometry_msgs/Pose cmd      # subsystem_buffer{type: port}
#geometry_msgs/Wrench force  # subsystem_buffer{type: port}
int32 status                # subsystem_buffer{type: port}

int32 INACTIVE=2
int32 ACTIVE=1
int32 SUCCESSFUL = 0
int32 INVALID_GOAL = -1
int32 OLD_HEADER_TIMESTAMP = -2
int32 PATH_TOLERANCE_VIOLATED = -3
int32 GOAL_TOLERANCE_VIOLATED = -4

