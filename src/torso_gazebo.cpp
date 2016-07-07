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
#include "rtt_rosclock/rtt_rosclock.h"

void TorsoGazebo::getJointPositionAndVelocity(Eigen::VectorXd &q, Eigen::VectorXd &dq) {
    std::string joint_names[1] = {"torso_0_joint"};
    for (int i=0; i<1; i++) {
        gazebo::physics::JointPtr joint = model_->GetJoint(joint_names[i]);
        q(i) = joint->GetAngle(0).Radian();
        dq(i) = joint->GetVelocity(0);
    }
}

void TorsoGazebo::getHeadJointPositionAndVelocity(Eigen::VectorXd &q, Eigen::VectorXd &dq) {
    std::string joint_names[2] = {"head_pan_joint", "head_tilt_joint"};
    for (int i=0; i<2; i++) {
        gazebo::physics::JointPtr joint = model_->GetJoint(joint_names[i]);
        q(i) = joint->GetAngle(0).Radian();
        dq(i) = joint->GetVelocity(0);
    }
}

void TorsoGazebo::setForces(const Eigen::VectorXd &t) {
    std::string joint_names[1] = {"torso_0_joint"};
    for (int i=0; i<1; i++) {
        model_->GetJoint(joint_names[i])->SetForce(0, t(i));
    }
}

KDL::Vector gz2kdl(const gazebo::math::Vector3 &v) {
    return KDL::Vector(v.x, v.y, v.z);
}

KDL::Frame gz2kdl(const gazebo::math::Pose &p) {
    return KDL::Frame(KDL::Rotation::Quaternion(p.rot.x,p.rot.y,p.rot.z,p.rot.w), gz2kdl(p.pos));
}

bool TorsoGazebo::gazeboConfigureHook(gazebo::physics::ModelPtr model) {

    if(model.get() == NULL) {
        std::cout << "TorsoGazebo::gazeboConfigureHook: the gazebo model is NULL" << std::endl;
        return false;
    }

    model_ = model;

    jc_ = new gazebo::physics::JointController(model);

    // head joints
    head_pan_joint_ = model->GetJoint("head_pan_joint");
    head_tilt_joint_ = model->GetJoint("head_tilt_joint");

    rtt_rosclock::set_sim_clock_activity(this);

    return true;
}

void TorsoGazebo::setJointsPID() {
    jc_->Reset();

    jc_->AddJoint(head_pan_joint_);
    jc_->SetPositionPID(head_pan_joint_->GetScopedName(), gazebo::common::PID(1.0, 0.5, 0.0, 0.1, -0.1, 1.0,-1.0));
    jc_->SetPositionTarget(head_pan_joint_->GetScopedName(), head_pan_joint_->GetAngle(0).Radian());

    jc_->AddJoint(head_tilt_joint_);
    jc_->SetPositionPID(head_tilt_joint_->GetScopedName(), gazebo::common::PID(1.0, 0.5, 0.0, 0.1, -0.1, 1.0,-1.0));
    jc_->SetPositionTarget(head_tilt_joint_->GetScopedName(), head_tilt_joint_->GetAngle(0).Radian());

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void TorsoGazebo::gazeboUpdateHook(gazebo::physics::ModelPtr model)
{
    Eigen::VectorXd q(1), dq(1);
    getJointPositionAndVelocity(q, dq);

    const double torso_gear = 158.0;
    const double torso_trans_mult = 20000.0 * torso_gear / (M_PI * 2.0);
    const double torso_motor_offset = -88524.0;
    const double torso_joint_offset = 0;
    const double torso_motor_constant = 0.00105;

    tmp_t_MotorPosition_out_ = (q(0) - torso_joint_offset) * torso_trans_mult + torso_motor_offset;
    tmp_t_MotorVelocity_out_ = dq(0) * torso_trans_mult;

    //
    // head
    //
    Eigen::VectorXd qh(2), dqh(2);
    getHeadJointPositionAndVelocity(qh, dqh);

    const double head_trans = 8000.0 * 100.0 / (M_PI * 2.0);
    tmp_hp_q_out_ = -qh(0) * head_trans;
    tmp_ht_q_out_ = qh(1) * head_trans;
    tmp_hp_v_out_ = dqh(0);
    tmp_ht_v_out_ = dqh(1);

    // exchange the data between Orocos and Gazebo
    {
        RTT::os::MutexLock lock(gazebo_mutex_);

        t_MotorPosition_out_ = tmp_t_MotorPosition_out_;
        t_MotorVelocity_out_ = tmp_t_MotorVelocity_out_;
        hp_q_out_ = tmp_hp_q_out_;
        ht_q_out_ = tmp_ht_q_out_;
        hp_v_out_ = tmp_hp_v_out_;
        ht_v_out_ = tmp_ht_v_out_;

        tmp_t_MotorCurrentCommand_in_ = t_MotorCurrentCommand_in_;
        tmp_hp_q_in_ = hp_q_in_;
        tmp_ht_q_in_ = ht_q_in_;
    }

    Eigen::VectorXd grav(1);
    grav.setZero();
    grav(0) += tmp_t_MotorCurrentCommand_in_ * torso_gear * torso_motor_constant;

    setForces(grav);

    // joint controller for the head
    jc_->SetPositionTarget(head_pan_joint_->GetScopedName(), -tmp_hp_q_in_ / head_trans);
    jc_->SetPositionTarget(head_tilt_joint_->GetScopedName(), tmp_ht_q_in_ / head_trans);

    jc_->Update();
}

