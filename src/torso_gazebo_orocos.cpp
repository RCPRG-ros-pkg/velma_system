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

#include "torso_gazebo.h"
#include <rtt/Logger.hpp>

using namespace RTT;

    void TorsoGazebo::updateHook() {
        Logger::In in("TorsoGazebo::updateHook");

        // Synchronize with gazeboUpdate()
        RTT::os::MutexLock lock(gazebo_mutex_);

        if (!data_valid_) {
            Logger::log() << Logger::Debug << "gazebo is not initialized" << Logger::endl;
            return;
        }
        else {
            Logger::log() << Logger::Debug << Logger::endl;
        }

        port_t_MotorPosition_out_.write(t_MotorPosition_out_);
        port_t_MotorVelocity_out_.write(t_MotorVelocity_out_);

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

    bool TorsoGazebo::startHook() {
      return true;
    }

    bool TorsoGazebo::configureHook() {
        setJointsPID();
        return true;
    }

