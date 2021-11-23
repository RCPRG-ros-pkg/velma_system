# Command for right end-effector in Cartesian impedance mode
CommandCartImp cart_r
bool cart_r_valid

# Command for left end-effector in Cartesian impedance mode
CommandCartImp cart_l
bool cart_l_valid

# Command for both arms and torso in joint impedance mode
CommandJntImp jnt
bool jnt_valid

# Command for head
CommandHead head
bool head_valid

# Command to switch to the relax behaviour
bool relax
bool relax_valid

# Command for the right hand - move fingers
barrett_hand_msgs/CommandHand handRight
bool handRight_valid

# Command for the right hand - reset
bool handRightReset
bool handRightReset_valid

# Command for the left hand - move fingers
barrett_hand_msgs/CommandHand handLeft
bool handLeft_valid

# Command for the left hand - reset
bool handLeftReset
bool handLeftReset_valid

# Command for head pan motor - enable, begin homing
CommandServo hp
bool hp_valid

# Command for head tilt motor - enable, begin homing
CommandServo ht
bool ht_valid

# Command for torso motor - enable
CommandServo t
bool t_valid

# Gravity compensation for a grasped object
bool objectGraspedRight
bool objectGraspedRight_valid

# Identification of mass of grasped object
bool objectGraspedLeft
bool objectGraspedLeft_valid

uint32 identificationMeasurementRightCmd
bool identificationMeasurementRightCmd_valid

uint32 identificationMeasurementLeftCmd
bool identificationMeasurementLeftCmd_valid

# Flag that allows collision of hands
int32 allow_hands_col
bool allow_hands_col_valid
