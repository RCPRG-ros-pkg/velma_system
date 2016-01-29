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

        dart_world_ = boost::dynamic_pointer_cast < gazebo::physics::DARTPhysics > ( gazebo::physics::get_world()->GetPhysicsEngine() ) -> GetDARTWorld();

        model_dart_ = boost::dynamic_pointer_cast < gazebo::physics::DARTModel >(model);
        if (model_dart_.get() == NULL) {
            std::cout << "BarrettHandGazebo::gazeboConfigureHook: the gazebo model is not a DART model" << std::endl;
            return false;
        }

        dart_sk_ = model_dart_->GetDARTSkeleton();

        return true;
    }

double BarrettHandGazebo::clip(double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void BarrettHandGazebo::gazeboUpdateHook(gazebo::physics::ModelPtr model)
{
    if (!model_dart_) {
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

    // spread joints
    const double vel_trap_angle = 5.0/180.0*3.1415;
    const double spread_dead_angle = 3.0/180.0*3.1415;

    double diff, force;
    int dof_idx, jnt_idx;
    int f1k1_dof_idx = 3;
    int f1k1_jnt_idx = 0;
    int f2k1_dof_idx = 3;
    int f2k1_jnt_idx = 3;

    double f1k1_force = joints_[f1k1_jnt_idx]->GetForce(0);
    double f2k1_force = joints_[f2k1_jnt_idx]->GetForce(0);
    double spread_force = f1k1_force + f2k1_force;

    if (std::fabs(spread_force) > 0.5) {
        status_out_ |= STATUS_OVERCURRENT4;
    }

    double f1k1_angle = joints_[f1k1_jnt_idx]->GetAngle(0).Radian();
    double f2k1_angle = joints_[f2k1_jnt_idx]->GetAngle(0).Radian();

    double f1k1_diff = clip( q_in_[f1k1_dof_idx] - joints_[f1k1_jnt_idx]->GetAngle(0).Radian(), -vel_trap_angle, vel_trap_angle);
    double f2k1_diff = clip( q_in_[f2k1_dof_idx] - joints_[f2k1_jnt_idx]->GetAngle(0).Radian(), -vel_trap_angle, vel_trap_angle);

    double spread_diff = f1k1_angle - f2k1_angle;

    double f1k1_vel = f1k1_diff - spread_diff / 2.0;
    double f2k1_vel = f2k1_diff + spread_diff / 2.0;

    if (std::fabs(f1k1_diff) < vel_trap_angle * 0.2 && std::fabs(f2k1_diff) < vel_trap_angle * 0.2) {
        status_out_ |= STATUS_IDLE4;
    }

    if (!jc_->SetVelocityTarget(joints_[f1k1_jnt_idx]->GetScopedName(), v_in_[f1k1_dof_idx] * f1k1_vel / vel_trap_angle)) {
        std::cout << "ERROR: BarrettHandGazebo::gazeboUpdateHook: jc_->SetVelocityTarget(" << joints_[f1k1_jnt_idx]->GetScopedName() << ")" << std::endl;
    }
    if (!jc_->SetVelocityTarget(joints_[f2k1_jnt_idx]->GetScopedName(), v_in_[f2k1_dof_idx] * f2k1_vel / vel_trap_angle)) {
        std::cout << "ERROR: BarrettHandGazebo::gazeboUpdateHook: jc_->SetVelocityTarget(" << joints_[f2k1_jnt_idx]->GetScopedName() << ")" << std::endl;
    }

    // finger joints
    int k2_dof_tab[3] = {0, 1, 2};
    int k2_jnt_tab[3] = {1, 4, 6};
    int k3_jnt_tab[3] = {2, 5, 7};
    int status_overcurrent_tab[3] = {STATUS_OVERCURRENT1, STATUS_OVERCURRENT2, STATUS_OVERCURRENT3};
    int status_idle_tab[3] = {STATUS_IDLE1, STATUS_IDLE2, STATUS_IDLE3};

    for (int i = 0; i < 3; i++) {
        double k2_diff;
        double k3_diff;

        if (!clutch_break_[i]) {
            k2_diff = clip( q_in_[k2_dof_tab[i]] - joints_[k2_jnt_tab[i]]->GetAngle(0).Radian(), -vel_trap_angle, vel_trap_angle);
            k3_diff = joints_[k2_jnt_tab[i]]->GetAngle(0).Radian() / 3.0 - joints_[k3_jnt_tab[i]]->GetAngle(0).Radian();
            jc_->SetVelocityTarget(joints_[k2_jnt_tab[i]]->GetScopedName(), v_in_[k2_dof_tab[i]] * k2_diff / vel_trap_angle);
            jc_->SetVelocityTarget(joints_[k3_jnt_tab[i]]->GetScopedName(), v_in_[k2_dof_tab[i]] * k3_diff / vel_trap_angle);
        }
        else {
        }

        double k2_force = joints_[k2_jnt_tab[i]]->GetForce(0);
        double k3_force = joints_[k3_jnt_tab[i]]->GetForce(0);
        if (std::fabs(k2_force) > 0.5 || std::fabs(k3_force) > 0.5) {
            status_out_ |= status_overcurrent_tab[i];
        }
        if (std::fabs(k2_diff) < vel_trap_angle * 0.2) {
            status_out_ |= status_idle_tab[i];
        }
    }

    jc_->Update();
}

