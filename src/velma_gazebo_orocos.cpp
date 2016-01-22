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
        port_t_JointPosition_out_.write(t_JointPosition_out_);
        port_r_JointVelocity_out_.write(r_JointVelocity_out_);
        port_l_JointVelocity_out_.write(l_JointVelocity_out_);
        port_t_JointVelocity_out_.write(t_JointVelocity_out_);

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

        if (port_t_JointTorqueCommand_in_.read(t_JointTorqueCommand_in_) == RTT::NewData) {
        }

        //
        // BarrettHand
        //
        port_rh_q_out_.write(rh_q_out_);
        port_lh_q_out_.write(lh_q_out_);
        port_rh_t_out_.write(rh_t_out_);
        port_lh_t_out_.write(lh_t_out_);

        port_rh_status_out_.write(rh_status_out_);
        port_lh_status_out_.write(lh_status_out_);

        if (port_rh_q_in_.read(rh_q_in_) == RTT::NewData) {
            std::cout << "rh_q_in_: new data " << rh_q_in_.transpose() << std::endl;
            rh_status_out_ = 0;
            rh_move_hand_ = true;
        }
        if (port_rh_v_in_.read(rh_v_in_) == RTT::NewData) {
            std::cout << "rh_v_in_: new data " << rh_v_in_.transpose() << std::endl;
        }
        if (port_rh_t_in_.read(rh_t_in_) == RTT::NewData) {
            std::cout << "rh_t_in_: new data " << rh_t_in_.transpose() << std::endl;
        }

        if (port_lh_q_in_.read(lh_q_in_) == RTT::NewData) {
            std::cout << "lh_q_in_: new data " << lh_q_in_.transpose() << std::endl;
            lh_status_out_ = 0;
            lh_move_hand_ = true;
        }
        port_lh_v_in_.read(lh_v_in_);
        port_lh_t_in_.read(lh_t_in_);

        //
        // head
        //
        if (port_hp_q_in_.read(hp_q_in_) == RTT::NewData) {
            std::cout << "head pan: " << hp_q_in_ << std::endl;
        }
        port_hp_v_in_.read(hp_v_in_);
        port_hp_c_in_.read(hp_c_in_);
        port_hp_q_out_.write(hp_q_out_);
        port_hp_v_out_.write(hp_v_out_);
        if (port_ht_q_in_.read(ht_q_in_) == RTT::NewData) {
            std::cout << "head pan: " << hp_q_in_ << std::endl;
        }
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

