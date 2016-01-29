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

#include "barrett_hand_gazebo.h"

    void BarrettHandGazebo::updateHook() {
        // Synchronize with gazeboUpdate()
        RTT::os::MutexLock lock(gazebo_mutex_);

        //
        // BarrettHand
        //
        port_q_out_.write(q_out_);
        port_t_out_.write(t_out_);

        port_status_out_.write(status_out_);

        if (port_q_in_.read(q_in_) == RTT::NewData) {
//            std::cout << "q_in_: new data " << q_in_.transpose() << std::endl;
            status_out_ = 0;
            move_hand_ = true;
        }
        port_v_in_.read(v_in_);
        port_t_in_.read(t_in_);
    }

    bool BarrettHandGazebo::startHook() {
      return true;
    }

    bool BarrettHandGazebo::configureHook() {

        if (prefix_.empty()) {
            std::cout << "ERROR: BarrettHandGazebo::configureHook: prefix is empty" << std::endl;
            return false;
        }

        std::string hand_joint_names[] = {"_HandFingerOneKnuckleOneJoint", "_HandFingerOneKnuckleTwoJoint", "_HandFingerOneKnuckleThreeJoint",
            "_HandFingerTwoKnuckleOneJoint", "_HandFingerTwoKnuckleTwoJoint", "_HandFingerTwoKnuckleThreeJoint",
            "_HandFingerThreeKnuckleTwoJoint", "_HandFingerThreeKnuckleThreeJoint" };

        for (int i = 0; i < 8; i++) {
            std::string name( prefix_ + hand_joint_names[i] );
            dart_sk_->getJoint(name)->setActuatorType( dart::dynamics::Joint::FORCE );
            gazebo::physics::JointPtr joint = model_->GetJoint(name);
            joints_.push_back(joint);
            dart::dynamics::Joint* joint_dart = dart_sk_->getJoint(name);
            joints_dart_.push_back( joint_dart );
            joint->SetEffortLimit(0, 1);
        }

        jc_ = new gazebo::physics::JointController(model_);

        for (int i = 0; i < 3; i++) {
            clutch_break_[i] = false;
        }

        for (int i = 0; i < 8; i++) {
            jc_->AddJoint(joints_[i]);
        }
        jc_->SetVelocityPID(joints_[0]->GetScopedName(), gazebo::common::PID(1.0, 0.0, 0.0, 0.0, 0.0, 1.0,-1.0));
        jc_->SetVelocityPID(joints_[3]->GetScopedName(), gazebo::common::PID(1.0, 0.0, 0.0, 0.0, 0.0, 1.0,-1.0));
        jc_->SetVelocityPID(joints_[1]->GetScopedName(), gazebo::common::PID(0.5, 0.0, 0.0, 0.0, 0.0, 0.5,-0.5));
        jc_->SetVelocityPID(joints_[4]->GetScopedName(), gazebo::common::PID(0.5, 0.0, 0.0, 0.0, 0.0, 0.5,-0.5));
        jc_->SetVelocityPID(joints_[6]->GetScopedName(), gazebo::common::PID(0.5, 0.0, 0.0, 0.0, 0.0, 0.5,-0.5));
        jc_->SetVelocityPID(joints_[2]->GetScopedName(), gazebo::common::PID(0.2, 0.0, 0.0, 0.0, 0.0, 0.2,-0.2));
        jc_->SetVelocityPID(joints_[5]->GetScopedName(), gazebo::common::PID(0.2, 0.0, 0.0, 0.0, 0.0, 0.2,-0.2));
        jc_->SetVelocityPID(joints_[7]->GetScopedName(), gazebo::common::PID(0.2, 0.0, 0.0, 0.0, 0.0, 0.2,-0.2));

        std::cout << "BarrettHandGazebo::configureHook(" << prefix_ << "): ok " << std::endl;
        return true;
    }

