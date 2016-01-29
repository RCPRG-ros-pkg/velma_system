#include "velma_gazebo.h"

    void VelmaGazebo::updateHook() {
        // Synchronize with gazeboUpdate()
        RTT::os::MutexLock lock(gazebo_mutex_);

        port_r_MassMatrix_out_.write(r_MassMatrix_out_);
        port_l_MassMatrix_out_.write(l_MassMatrix_out_);
        port_r_GravityTorque_out_.write(r_GravityTorque_out_);
        port_l_GravityTorque_out_.write(l_GravityTorque_out_);
        port_r_JointTorque_out_.write(r_JointTorque_out_);
        port_l_JointTorque_out_.write(l_JointTorque_out_);
        port_r_JointPosition_out_.write(r_JointPosition_out_);
        port_l_JointPosition_out_.write(l_JointPosition_out_);
        port_t_MotorPosition_out_.write(t_MotorPosition_out_);
        port_r_JointVelocity_out_.write(r_JointVelocity_out_);
        port_l_JointVelocity_out_.write(l_JointVelocity_out_);
        port_t_MotorVelocity_out_.write(t_MotorVelocity_out_);

        // FRI comm state
        r_FRIState_out_.quality = FRI_QUALITY_PERFECT;
        l_FRIState_out_.quality = FRI_QUALITY_PERFECT;
        r_FRIState_out_.state = FRI_STATE_CMD;      // FRI_STATE_MON
        l_FRIState_out_.state = FRI_STATE_CMD;
        port_r_FRIState_out_.write(r_FRIState_out_);
        port_l_FRIState_out_.write(l_FRIState_out_);

        // FRI robot state
        r_RobotState_out_.power = 0x7F;
        r_RobotState_out_.error = 0;
        r_RobotState_out_.warning = 0;
        r_RobotState_out_.control == FRI_CTRL_POSITION;     // FRI_CTRL_CART_IMP, FRI_CTRL_JNT_IMP
        l_RobotState_out_.power = 0x7F;
        l_RobotState_out_.error = 0;
        l_RobotState_out_.warning = 0;
        l_RobotState_out_.control == FRI_CTRL_POSITION;     // FRI_CTRL_CART_IMP, FRI_CTRL_JNT_IMP
        port_r_RobotState_out_.write(r_RobotState_out_);
        port_l_RobotState_out_.write(l_RobotState_out_);

        if (port_r_KRL_CMD_in_.read(r_KRL_CMD_in_) == RTT::NewData) {
            if (r_KRL_CMD_in_.data == 1) {
                r_command_mode_ = true;
            }
            else if (r_KRL_CMD_in_.data == 2) {
                r_command_mode_ = false;
            }
        }

        if (port_l_KRL_CMD_in_.read(l_KRL_CMD_in_) == RTT::NewData) {
            if (l_KRL_CMD_in_.data == 1) {
                l_command_mode_ = true;
            }
            else if (l_KRL_CMD_in_.data == 2) {
                l_command_mode_ = false;
            }
        }

        if (port_r_JointTorqueCommand_in_.read(r_JointTorqueCommand_in_) == RTT::NewData) {
        }

        if (port_l_JointTorqueCommand_in_.read(l_JointTorqueCommand_in_) == RTT::NewData) {
        }

        if (port_t_MotorCurrentCommand_in_.read(t_MotorCurrentCommand_in_) == RTT::NewData) {
        }

        //
        // head
        //
        port_hp_q_in_.read(hp_q_in_);
        port_hp_v_in_.read(hp_v_in_);
        port_hp_c_in_.read(hp_c_in_);
        port_hp_q_out_.write(hp_q_out_);
        port_hp_v_out_.write(hp_v_out_);
        port_ht_q_in_.read(ht_q_in_);
        port_ht_v_in_.read(ht_v_in_);
        port_ht_c_in_.read(ht_c_in_);
        port_ht_q_out_.write(ht_q_out_);
        port_ht_v_out_.write(ht_v_out_);
    }

    bool VelmaGazebo::startHook() {
      return true;
    }

    bool VelmaGazebo::configureHook() {
        std::cout << "VelmaGazebo::configureHook: ok" << std::endl;
        return true;
    }

