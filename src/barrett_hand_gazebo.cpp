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

    bool BarrettHandGazebo::gazeboConfigureHook(gazebo::physics::ModelPtr model) {

        if(model.get() == NULL) {
            std::cout << "BarrettHandGazebo::gazeboConfigureHook: the gazebo model is NULL" << std::endl;
            return false;
        }

        model_ = model;
/*
        model_dart_ = boost::dynamic_pointer_cast < gazebo::physics::DARTModel >(model);
        if (model_dart_.get() == NULL) {
            std::cout << "BarrettHandGazebo::gazeboConfigureHook: the gazebo model is not a DART model" << std::endl;
            return false;
        }

        dart_sk_ = model_dart_->GetDARTSkeleton();
*/
        jc_ = new gazebo::physics::JointController(model_);

        return true;
    }

double BarrettHandGazebo::clip(double n, double lower, double upper) const {
    return std::max(lower, std::min(n, upper));
}

double BarrettHandGazebo::getFingerAngle(int fidx) const {
    const int k2_jnt_tab[3] = {1, 4, 6};
    const int k3_jnt_tab[3] = {2, 5, 7};

    double k2_pos = joints_[k2_jnt_tab[fidx]]->GetAngle(0).Radian();
    double k3_pos = joints_[k3_jnt_tab[fidx]]->GetAngle(0).Radian();

    return k2_pos + (k3_pos - k2_pos / 3.0);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void BarrettHandGazebo::gazeboUpdateHook(gazebo::physics::ModelPtr model)
{
//    if (!model_dart_ || joints_.size() == 0) {
    if (joints_.size() == 0) {
        return;
    }

    RTT::os::MutexTryLock trylock(gazebo_mutex_);
    if(!trylock.isSuccessful()) {
        return;
    }

    //
    // BarrettHand
    //

    const double force_factor = 1000.0;
    // joint position
    for (int i = 0; i < 8; i++) {
        q_out_(i) = joints_[i]->GetAngle(0).Radian();
    }

    t_out_[0] = t_out_[3] = joints_[0]->GetForce(0)*force_factor;
    t_out_[1] = t_out_[2] = joints_[1]->GetForce(0)*force_factor;
    t_out_[4] = t_out_[5] = joints_[4]->GetForce(0)*force_factor;
    t_out_[6] = t_out_[7] = joints_[6]->GetForce(0)*force_factor;

    int f1k1_dof_idx = 3;
    int f1k1_jnt_idx = 0;
    int f2k1_dof_idx = 3;
    int f2k1_jnt_idx = 3;
    double mean_spread = (joints_[f1k1_jnt_idx]->GetAngle(0).Radian() + joints_[f2k1_jnt_idx]->GetAngle(0).Radian()) / 2.0;
    if (move_hand_) {
        move_hand_ = false;
        spread_int_ = mean_spread;
        finger_int_[0] = getFingerAngle(0);
        finger_int_[1] = getFingerAngle(1);
        finger_int_[2] = getFingerAngle(2);
        status_out_ = 0;
        std::cout << "move hand" << std::endl;
    }
//    std::cout << "status_out_: " << status_out_ << std::endl;

    if ((status_out_&STATUS_OVERCURRENT4) == 0 && (status_out_&STATUS_IDLE4) == 0) {
        // spread joints
        if (spread_int_ > q_in_[f1k1_dof_idx]) {
            spread_int_ -= v_in_[f1k1_dof_idx] * 0.001;
            if (spread_int_ <= q_in_[f1k1_dof_idx]) {
                status_out_ |= STATUS_IDLE4;
                std::cout << "spread idle" << std::endl;
            }
        }
        else if (spread_int_ < q_in_[f1k1_dof_idx]) {
            spread_int_ += v_in_[f1k1_dof_idx] * 0.001;
            if (spread_int_ >= q_in_[f1k1_dof_idx]) {
                status_out_ |= STATUS_IDLE4;
                std::cout << "spread idle" << std::endl;
            }
        }

//        std::cout << "spread_int_: " << spread_int_ << std::endl;
        double f1k1_force = joints_[f1k1_jnt_idx]->GetForce(0);
        double f2k1_force = joints_[f2k1_jnt_idx]->GetForce(0);
        double spread_force = f1k1_force + f2k1_force;

        if (std::fabs(spread_force) > 0.5) {
            status_out_ |= STATUS_OVERCURRENT4;
            std::cout << "spread overcurrent" << std::endl;
            jc_->SetPositionTarget(joints_[f1k1_jnt_idx]->GetScopedName(), mean_spread);
            jc_->SetPositionTarget(joints_[f2k1_jnt_idx]->GetScopedName(), mean_spread);
        }
        else {
            if (!jc_->SetPositionTarget(joints_[f1k1_jnt_idx]->GetScopedName(), spread_int_)) {
                std::cout << "ERROR: BarrettHandGazebo::gazeboUpdateHook: jc_->SetPositionTarget(" << joints_[f1k1_jnt_idx]->GetScopedName() << ")" << std::endl;
            }
            if (!jc_->SetPositionTarget(joints_[f2k1_jnt_idx]->GetScopedName(), spread_int_)) {
                std::cout << "ERROR: BarrettHandGazebo::gazeboUpdateHook: jc_->SetPositionTarget(" << joints_[f2k1_jnt_idx]->GetScopedName() << ")" << std::endl;
            }
        }
    }

    // finger joints
    const int k2_dof_tab[3] = {0, 1, 2};
    const int k2_jnt_tab[3] = {1, 4, 6};
    const int k3_jnt_tab[3] = {2, 5, 7};
    const int status_overcurrent_tab[3] = {STATUS_OVERCURRENT1, STATUS_OVERCURRENT2, STATUS_OVERCURRENT3};
    const int status_idle_tab[3] = {STATUS_IDLE1, STATUS_IDLE2, STATUS_IDLE3};
    for (int fidx = 0; fidx < 3; fidx++) {
        int k2_dof = k2_dof_tab[fidx];
        int k2_jnt = k2_jnt_tab[fidx];
        int k3_jnt = k3_jnt_tab[fidx];
        int STATUS_OVERCURRENTi = status_overcurrent_tab[fidx];
        int STATUS_IDLEi = status_idle_tab[fidx];
        bool is_opening = false;
        if ((status_out_&STATUS_OVERCURRENTi) == 0 && (status_out_&STATUS_IDLEi) == 0) {
            if (finger_int_[fidx] > q_in_[k2_dof]) {
                finger_int_[fidx] -= v_in_[k2_dof] * 0.001;
                is_opening = true;
                if (finger_int_[fidx] <= q_in_[k2_dof]) {
                    status_out_ |= STATUS_IDLEi;
                    std::cout << "finger " << fidx << " idle" << std::endl;
                }
            }
            else if (finger_int_[fidx] < q_in_[k2_dof]) {
                finger_int_[fidx] += v_in_[k2_dof] * 0.001;
                is_opening = false;
                if (finger_int_[fidx] >= q_in_[k2_dof]) {
                    status_out_ |= STATUS_IDLEi;
                    std::cout << "finger " << fidx << " idle" << std::endl;
                }
            }

            double k2_force = joints_[k2_jnt]->GetForce(0);
            double k3_force = joints_[k3_jnt]->GetForce(0);
            double k2_angle = joints_[k2_jnt]->GetAngle(0).Radian();
            double k3_angle = joints_[k3_jnt]->GetAngle(0).Radian();

            if (!is_opening && std::fabs(k2_force) > 0.25) {
                clutch_break_angle_[fidx] = k2_angle;
                clutch_break_[fidx] = true;
                std::cout << "finger " << fidx << " clutch is broken" << std::endl;
            }

            double k3_angle_dest;
            double k2_angle_dest;
            if (std::fabs(k2_force) + std::fabs(k3_force) > 0.5) {
                status_out_ |= STATUS_OVERCURRENTi;
                std::cout << "finger " << fidx << " overcurrent" << std::endl;
                k2_angle_dest = k2_angle;
                k3_angle_dest = k3_angle;
            }
            else if (clutch_break_[fidx]) {
                k3_angle_dest = finger_int_[fidx] - clutch_break_angle_[fidx] * 2.0 / 3.0;
                k2_angle_dest = clutch_break_angle_[fidx];
                if (is_opening && k3_angle_dest < clutch_break_angle_[fidx] / 3.0) {
                    k2_angle_dest = finger_int_[fidx];
                    k3_angle_dest = finger_int_[fidx]/3;
                    if (k2_angle < 0.03) {
                        clutch_break_[fidx] = false;
                        std::cout << "finger " << fidx << " clutch is restored" << std::endl;
                    }
                }
            }
            else {
                k2_angle_dest = finger_int_[fidx];
                k3_angle_dest = finger_int_[fidx]/3;
            }

//            std::cout << "finger " << fidx << "  dest: " << k2_angle_dest << "   " << k3_angle_dest << "   cur: " << joints_[k2_jnt]->GetAngle(0).Radian()
//                << "  " << joints_[k3_jnt]->GetAngle(0).Radian() << std::endl;
            jc_->SetPositionTarget(joints_[k2_jnt]->GetScopedName(), k2_angle_dest);
            jc_->SetPositionTarget(joints_[k3_jnt]->GetScopedName(), k3_angle_dest);
        }

        // fingers may break if the force is too big
        if (too_big_force_counter_[fidx] < 100) {
            gazebo::physics::JointWrench k2_wrench = joints_[k2_jnt]->GetForceTorque(0);
            gazebo::physics::JointWrench k3_wrench = joints_[k3_jnt]->GetForceTorque(0);
            if (k2_wrench.body1Force.GetLength() > 40.0 || k2_wrench.body1Torque.GetLength() > 8.0 ||
                k3_wrench.body1Force.GetLength() > 40.0 || k3_wrench.body1Torque.GetLength() > 6.0) {
                too_big_force_counter_[fidx]++;
            }
            else {
                too_big_force_counter_[fidx] = 0;
            }
        }
        if (too_big_force_counter_[fidx] == 100) {
//            joints_dart_[k2_jnt]->setPositionLimited(false);
//            joints_dart_[k3_jnt]->setPositionLimited(false);
            jc_->SetPositionPID(joints_[k2_jnt]->GetScopedName(), gazebo::common::PID());
            jc_->SetPositionPID(joints_[k3_jnt]->GetScopedName(), gazebo::common::PID());
            std::cout << "finger " << fidx << " is broken: " << std::endl;//k2_wrench.body1Force.GetLength() << " " << k2_wrench.body1Torque.GetLength() << " " << k3_wrench.body1Force.GetLength() << " " << k3_wrench.body1Torque.GetLength() << std::endl;
            too_big_force_counter_[fidx]++;
        }
    }

    jc_->Update();

}

