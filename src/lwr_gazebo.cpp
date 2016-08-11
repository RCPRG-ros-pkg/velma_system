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

#include "lwr_gazebo.h"
#include "rtt_rosclock/rtt_rosclock.h"

void LWRGazebo::getExternalForces(Eigen::VectorXd &q) {
    for (int i=0; i<joint_names_.size(); i++) {
        q(i) = model_->GetJoint(joint_names_[i])->GetForce(0);
    }
}

void LWRGazebo::getJointPositionAndVelocity(Eigen::VectorXd &q, Eigen::VectorXd &dq) {
    for (int i=0; i<joint_names_.size(); i++) {
        gazebo::physics::JointPtr joint = model_->GetJoint(joint_names_[i]);
        q(i) = joint->GetAngle(0).Radian();
        dq(i) = joint->GetVelocity(0);
    }
}

void LWRGazebo::setForces(const Eigen::VectorXd &t) {
    for (int i=0; i<joint_names_.size(); i++) {
        model_->GetJoint(joint_names_[i])->SetForce(0, t(i));
    }
}

KDL::Vector gz2kdl(const gazebo::math::Vector3 &v) {
    return KDL::Vector(v.x, v.y, v.z);
}

KDL::Frame gz2kdl(const gazebo::math::Pose &p) {
    return KDL::Frame(KDL::Rotation::Quaternion(p.rot.x,p.rot.y,p.rot.z,p.rot.w), gz2kdl(p.pos));
}

void LWRGazebo::getGravComp(Eigen::VectorXd &t) {
    KDL::Vector gr = gz2kdl( gazebo::physics::get_world()->GetPhysicsEngine()->GetGravity() );

    gazebo::physics::LinkPtr link = model_->GetLink(name_ + "_arm_7_link");
    gazebo::physics::JointPtr joint = model_->GetJoint(name_ + "_arm_6_joint");
    KDL::Frame T_W_L7 = gz2kdl( link->GetWorldPose() );
    KDL::Vector cog = T_W_L7 * KDL::Vector(tool_.com.x, tool_.com.y, tool_.com.z);
    KDL::Vector r = cog-gz2kdl( joint->GetWorldPose().pos );
    double mass = tool_.m;
    KDL::Vector torque = r * (mass * gr);
    KDL::Vector axis = gz2kdl( joint->GetGlobalAxis(0) );
    t(6) = KDL::dot(axis, torque);

    for (int i=6; i>0; i--) {
        link = model_->GetLink(name_ + std::string("_arm_") + std::to_string(i) + "_link");
        joint = model_->GetJoint(name_ + std::string("_arm_") + std::to_string(i-1) + "_joint");
        cog = (cog*mass + gz2kdl(link->GetWorldCoGPose().pos)*link->GetInertial()->GetMass()) / (mass+link->GetInertial()->GetMass());
        mass += link->GetInertial()->GetMass();
        r = cog-gz2kdl( joint->GetWorldPose().pos );
        torque = r * (mass * gr);
        axis = gz2kdl( joint->GetGlobalAxis(0) );
        t(i-1) = KDL::dot(axis, torque);
    }

    t = -t;
}

bool LWRGazebo::gazeboConfigureHook(gazebo::physics::ModelPtr model) {

    if(model.get() == NULL) {
        std::cout << "LWRGazebo::gazeboConfigureHook: the gazebo model is NULL" << std::endl;
        return false;
    }

    model_ = model;

    rtt_rosclock::set_sim_clock_activity(this);

    return true;
}

void LWRGazebo::setInitialPosition(const std::map<std::string, double> &init_q) {
    init_q_map_ = init_q;
    std::vector<gazebo::physics::JointPtr>  joints = model_->GetJoints();

    for (std::map<std::string, double>::const_iterator it = init_q.begin(); it != init_q.end(); it++) {
        gazebo::physics::JointPtr joint = model_->GetJoint( it->first );
        gazebo::physics::DARTJointPtr joint_dart = boost::dynamic_pointer_cast < gazebo::physics::DARTJoint > ( joint );
        if (joint_dart != NULL) {
            joint_dart->GetDARTJoint()->setPosition(0, it->second);
        }
        joint->SetPosition(0, it->second);
        joint->SetVelocity(0, 0);
    }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void LWRGazebo::gazeboUpdateHook(gazebo::physics::ModelPtr model)
{
    // mass matrix
    mm_->updatePoses(model);
    const Eigen::MatrixXd &mm = mm_->getMassMatrix();

    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            tmp_MassMatrix_out_(i,j) = mm(i,j);
        }
    }

    // gravity forces
    Eigen::VectorXd grav(7);
    grav.setZero();
    getGravComp(grav);

    // gravity forces
    for (int i = 0; i < 7; i++) {
        tmp_GravityTorque_out_(i) = grav(i);
    }

    Eigen::VectorXd ext_f(7);
    getExternalForces(ext_f);

    // external forces
    for (int i = 0; i < 7; i++) {
        tmp_JointTorque_out_(i) = ext_f[i] - grav(i);
    }

    Eigen::VectorXd q(7), dq(7);
    getJointPositionAndVelocity(q, dq);

    // joint position
    for (int i = 0; i < 7; i++) {
        tmp_JointPosition_out_(i) = q(i);
    }

    // joint velocity
    for (int i = 0; i < 7; i++) {
        tmp_JointVelocity_out_(i) = dq(i);
    }

/*
    // TODO
    // calculate the wrench on the wrist
    Eigen::Isometry3d ET_WO_Wr = right_tool_bn_dart_->getTransform();
    Eigen::Vector3d grav_Wr = ET_WO_Wr.inverse().rotation() * Eigen::Vector3d(0,0,-9.81);
    KDL::Wrench grav_reaction_Wr = KDL::Frame(right_tool_com_W_) * KDL::Wrench(KDL::Vector(grav_Wr(0), grav_Wr(1), grav_Wr(2)), KDL::Vector());
    Eigen::Vector6d wr_Wr = right_tool_bn_dart_->getBodyForce();
    KDL::Wrench wrr_Wr = KDL::Wrench( -KDL::Vector(wr_Wr(3), wr_Wr(4), wr_Wr(5)), -KDL::Vector(wr_Wr(0), wr_Wr(1), wr_Wr(2)) ) - grav_reaction_Wr;
    tmp_r_CartesianWrench_out_.force.x = wrr_Wr.force.x();
    tmp_r_CartesianWrench_out_.force.y = wrr_Wr.force.y();
    tmp_r_CartesianWrench_out_.force.z = wrr_Wr.force.z();
    tmp_r_CartesianWrench_out_.torque.x = wrr_Wr.torque.x();
    tmp_r_CartesianWrench_out_.torque.y = wrr_Wr.torque.y();
    tmp_r_CartesianWrench_out_.torque.z = wrr_Wr.torque.z();

    Eigen::Isometry3d ET_WO_Wl = left_tool_bn_dart_->getTransform();
    Eigen::Vector3d grav_Wl = ET_WO_Wl.inverse().rotation() * Eigen::Vector3d(0,0,-9.81);
    KDL::Wrench grav_reaction_Wl = KDL::Frame(left_tool_com_W_) * KDL::Wrench(KDL::Vector(grav_Wl(0), grav_Wl(1), grav_Wl(2)), KDL::Vector());
    Eigen::Vector6d wr_Wl = left_tool_bn_dart_->getBodyForce();
    KDL::Wrench wrr_Wl = KDL::Wrench( -KDL::Vector(wr_Wl(3), wr_Wl(4), wr_Wl(5)), -KDL::Vector(wr_Wl(0), wr_Wl(1), wr_Wl(2)) ) - grav_reaction_Wl;
    tmp_l_CartesianWrench_out_.force.x = wrr_Wl.force.x();
    tmp_l_CartesianWrench_out_.force.y = wrr_Wl.force.y();
    tmp_l_CartesianWrench_out_.force.z = wrr_Wl.force.z();
    tmp_l_CartesianWrench_out_.torque.x = wrr_Wl.torque.x();
    tmp_l_CartesianWrench_out_.torque.y = wrr_Wl.torque.y();
    tmp_l_CartesianWrench_out_.torque.z = wrr_Wl.torque.z();
*/

    bool tmp_command_mode;

    // exchange the data between Orocos and Gazebo
    {
        RTT::os::MutexLock lock(gazebo_mutex_);

        MassMatrix_out_ = tmp_MassMatrix_out_;
        GravityTorque_out_ = tmp_GravityTorque_out_;
        JointTorque_out_ = tmp_JointTorque_out_;
        JointPosition_out_ = tmp_JointPosition_out_;
        JointVelocity_out_ = tmp_JointVelocity_out_;
        CartesianWrench_out_ = tmp_CartesianWrench_out_;
        tmp_JointTorqueCommand_in_ = JointTorqueCommand_in_;
        tmp_command_mode = command_mode_;
        data_valid_ = true;
    }

    // torque command
    if (tmp_command_mode) {
        for (int i = 0; i < 7; i++) {
            grav(i) += tmp_JointTorqueCommand_in_(i);
        }
    }
    else {
        for (int i = 0; i < joint_names_.size(); i++) {
            grav(i) += 10.0 * (init_q_map_[joint_names_[i]] - tmp_JointPosition_out_(i));
        }
    }

    setForces(grav);
}

