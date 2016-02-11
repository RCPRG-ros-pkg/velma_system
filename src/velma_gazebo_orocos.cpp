/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "velma_gazebo.h"

    void VelmaGazebo::updateHook() {
//        std::cout << "VelmaGazebo::updateHook" << std::endl;

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
                setJointsEnabledPID();
            }
            else if (r_KRL_CMD_in_.data == 2) {
                r_command_mode_ = false;
                setJointsDisabledPID();
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

        port_r_CartesianWrench_out_.write(r_CartesianWrench_out_);
        port_l_CartesianWrench_out_.write(l_CartesianWrench_out_);

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

