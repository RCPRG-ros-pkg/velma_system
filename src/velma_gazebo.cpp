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

bool VelmaGazebo::gazeboConfigureHook(gazebo::physics::ModelPtr model) {

        if(model.get() == NULL) {
            std::cout << "VelmaGazebo::gazeboConfigureHook: the gazebo model is NULL" << std::endl;
            return false;
        }

        model_ = model;

        dart_world_ = boost::dynamic_pointer_cast < gazebo::physics::DARTPhysics > ( gazebo::physics::get_world()->GetPhysicsEngine() ) -> GetDARTWorld();

        model_dart_ = boost::dynamic_pointer_cast < gazebo::physics::DARTModel >(model);
        if (model_dart_.get() == NULL) {
            std::cout << "VelmaGazebo::gazeboConfigureHook: the gazebo model is not a DART model" << std::endl;
            return false;
        }

        dart_sk_ = model_dart_->GetDARTSkeleton();

        jc_ = new gazebo::physics::JointController(model);

        std::string srdf;
        ros::param::get("/robot_semantic_description", srdf);
        dart::collision::CollisionDetector* detector = dart_world_->getConstraintSolver()->getCollisionDetector();

        std::vector<std::pair<std::string, std::string> > disabled_collisions;
        parseSRDF(srdf, disabled_collisions);
        for (std::vector<std::pair<std::string, std::string> >::const_iterator it = disabled_collisions.begin(); it != disabled_collisions.end(); it++) {
            dart::dynamics::BodyNode *bn1 = dart_sk_->getBodyNode(it->first);
            dart::dynamics::BodyNode *bn2 = dart_sk_->getBodyNode(it->second);
            detector->disablePair(bn1, bn2);
        }
        dart_sk_->enableSelfCollision(false);

        // fill in gazebo joints pointer vectors

        // right arm joints
        for(unsigned int i = 0; i < 7; i++) {
            std::string joint_name = std::string("right_arm_") + std::to_string(i) + "_joint";
            gazebo::physics::JointPtr joint = model->GetJoint(joint_name);
            if (joint) {
                gazebo::physics::DARTJointPtr joint_dart = boost::dynamic_pointer_cast < gazebo::physics::DARTJoint > ( joint );
                this->r_joints_.push_back(joint);
                r_dart_joints_.push_back( joint_dart->GetDARTJoint() );
                r_indices_.push_back(r_dart_joints_.back()->getIndexInSkeleton(0));
            }
            else {
                RTT::log(RTT::Error) << "A joint named " << joint_name.c_str() << " is not part of Mechanism Controlled joints." << RTT::endlog();
                return false;
            }
        }

        // left arm joints
        for(unsigned int i = 0; i < 7; i++) {
            std::string joint_name = std::string("left_arm_") + std::to_string(i) + "_joint";
            gazebo::physics::JointPtr joint = model->GetJoint(joint_name);
            if (joint) {
                gazebo::physics::DARTJointPtr joint_dart = boost::dynamic_pointer_cast < gazebo::physics::DARTJoint > ( joint );
                this->l_joints_.push_back(joint);
                l_dart_joints_.push_back( joint_dart->GetDARTJoint() );
                l_indices_.push_back(l_dart_joints_.back()->getIndexInSkeleton(0));
            }
            else {
                RTT::log(RTT::Error) << "A joint named " << joint_name.c_str() << " is not part of Mechanism Controlled joints." << RTT::endlog();
                return false;
            }
        }

        // torso joints
        for(unsigned int i = 0; i < 1; i++) {
            std::string joint_name = std::string("torso_") + std::to_string(i) + "_joint";
            gazebo::physics::JointPtr joint = model->GetJoint(joint_name);     
            if (joint) {
                gazebo::physics::DARTJointPtr joint_dart = boost::dynamic_pointer_cast < gazebo::physics::DARTJoint > ( joint );
                this->t_joints_.push_back(joint);
                t_dart_joints_.push_back( joint_dart->GetDARTJoint() );
                t_indices_.push_back(t_dart_joints_.back()->getIndexInSkeleton(0));
            }
            else {
                RTT::log(RTT::Error) << "A joint named " << joint_name.c_str() << " is not part of Mechanism Controlled joints." << RTT::endlog();
                return false;
            }
        }

        // head joints
        head_pan_joint_ = model->GetJoint("head_pan_joint");
        head_tilt_joint_ = model->GetJoint("head_tilt_joint");

        // print DART collisions
        for (int bidx = 0; bidx < dart_sk_->getNumBodyNodes(); bidx++) {
            dart::dynamics::BodyNode *bn = dart_sk_->getBodyNode(bidx);
            std::cout << "BodyNode " << bidx << "  " << bn->getName() << std::endl;
            int num_sh = bn->getNumCollisionShapes();
            for (int i = 0; i < num_sh; i++) {
                dart::dynamics::Shape *sh = bn->getCollisionShape(i);
                if (sh == NULL) {
                    std::cout << "   shape " << i << "  is NULL" << std::endl;
                    continue;
                }
                if (sh->getShapeType() == dart::dynamics::Shape::MESH) {
                    dart::dynamics::MeshShape *msh = static_cast<dart::dynamics::MeshShape*>(sh);
                    Eigen::Quaterniond q(sh->getLocalTransform().rotation());
                    std::cout << "   shape " << i << "  is mesh:  " << sh->getOffset().transpose() << "  " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " " << std::endl;
                }
                else {
                    Eigen::Quaterniond q(sh->getLocalTransform().rotation());
                    std::cout << "   shape " << i << "  is not mesh:  " << sh->getOffset().transpose() << "  " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " " << std::endl;
                }
            }
        }

        // print Gazebo collisions
        for (gazebo::physics::Link_V::const_iterator it = model->GetLinks().begin(); it != model->GetLinks().end(); it++) {
            std::cout << "link: " << (*it)->GetName() << std::endl;
            int col_count = (*it)->GetCollisions().size();
            for (int i = 0; i < col_count; i++) {
                gazebo::math::Vector3 pos = (*it)->GetCollisions()[i]->GetRelativePose().pos;
                gazebo::math::Quaternion q = (*it)->GetCollisions()[i]->GetRelativePose().rot;
                std::cout << "   collision " << (*it)->GetCollisions()[i]->GetName() << "  " << pos.x << " " << pos.y << " " << pos.z << "  " << q.x << " " << q.y << " " << q.z << " " << q.w << " " << std::endl; 
            }
        }

    for (int i = 0; i < dart_sk_->getNumBodyNodes(); i++) {
        dart::dynamics::BodyNode *bn = dart_sk_->getBodyNode(i);
        std::cout << bn->getName() << "  " << bn->getMass() << std::endl;
    }

    double mass = 0.0;
    Eigen::Isometry3d ET_WO_W = dart_sk_->getBodyNode("right_arm_7_link")->getTransform();
    Eigen::Vector3d com(0,0,0);
    for (int i = 0; i < dart_sk_->getNumBodyNodes(); i++) {
        dart::dynamics::BodyNode *bn = dart_sk_->getBodyNode(i);
        if (bn->getName().find("right_Hand") == 0 || bn->getName().find("right_arm_7_link") == 0) {
            mass += bn->getMass();
            Eigen::Isometry3d ET_WO_L = bn->getTransform();
            Eigen::Isometry3d ET_W_L = ET_WO_W.inverse() * ET_WO_L;
            com += ET_W_L * bn->getLocalCOM() * bn->getMass();            
        }
    }
    com /= mass;
    std::cout << "mass: " << mass << "  COM: " << com.transpose() << std::endl;

//BodyNode::setMass(double _mass)
//setLocalCOM

    r_force_prev_.resize(7);
    l_force_prev_.resize(7);

    setInitialPosition();

//    setJointsDisabledPID();
    setJointsEnabledPID();

    return true;
}

void VelmaGazebo::setInitialPosition() {
    double angle = 90.0/180.0*3.1415;

    r_dart_joints_[1]->setPosition(0, -angle);
    l_dart_joints_[1]->setPosition(0, angle);

    r_dart_joints_[2]->setPosition(0, angle);
    l_dart_joints_[2]->setPosition(0, -angle);

    r_dart_joints_[3]->setPosition(0, angle);
    l_dart_joints_[3]->setPosition(0, -angle);

    r_dart_joints_[5]->setPosition(0, -angle);
    l_dart_joints_[5]->setPosition(0, angle);
}

void VelmaGazebo::setJointsDisabledPID() {
    jc_->Reset();
    for (int i = 0; i < r_dart_joints_.size(); i++) {
        jc_->AddJoint(r_joints_[i]);
        jc_->SetPositionPID(r_joints_[i]->GetScopedName(), gazebo::common::PID(10.0, 0, 0.0, 1.1, -1.1, 10.0,-10.0));
        jc_->SetPositionTarget(r_joints_[i]->GetScopedName(), r_dart_joints_[i]->getPosition(0));
    }

    for (int i = 0; i < l_dart_joints_.size(); i++) {
        jc_->AddJoint(l_joints_[i]);
        jc_->SetPositionPID(l_joints_[i]->GetScopedName(), gazebo::common::PID(10.0, 0, 0.0, 1.1, -1.1, 10.0,-10.0));
        jc_->SetPositionTarget(l_joints_[i]->GetScopedName(), l_dart_joints_[i]->getPosition(0));
    }

    jc_->AddJoint(head_pan_joint_);
    jc_->SetPositionPID(head_pan_joint_->GetScopedName(), gazebo::common::PID(1.0, 0.5, 0.0, 0.1, -0.1, 1.0,-1.0));
    jc_->SetPositionTarget(head_pan_joint_->GetScopedName(), head_pan_joint_->GetAngle(0).Radian());

    jc_->AddJoint(head_tilt_joint_);
    jc_->SetPositionPID(head_tilt_joint_->GetScopedName(), gazebo::common::PID(1.0, 0.5, 0.0, 0.1, -0.1, 1.0,-1.0));
    jc_->SetPositionTarget(head_tilt_joint_->GetScopedName(), head_tilt_joint_->GetAngle(0).Radian());

//    jc_->AddJoint(head_tilt_joint_);
//    jc_->SetPositionPID(head_tilt_joint_->GetScopedName(), gazebo::common::PID(1.0, 0.5, 0.0, 0.1, -0.1, 1.0,-1.0));
}

void VelmaGazebo::setJointsEnabledPID() {
    jc_->Reset();

    jc_->AddJoint(head_pan_joint_);
    jc_->SetPositionPID(head_pan_joint_->GetScopedName(), gazebo::common::PID(1.0, 0.5, 0.0, 0.1, -0.1, 1.0,-1.0));

    jc_->AddJoint(head_tilt_joint_);
    jc_->SetPositionPID(head_tilt_joint_->GetScopedName(), gazebo::common::PID(1.0, 0.5, 0.0, 0.1, -0.1, 1.0,-1.0));
}

double VelmaGazebo::clip(double n, double lower, double upper) const {
  return std::max(lower, std::min(n, upper));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void VelmaGazebo::gazeboUpdateHook(gazebo::physics::ModelPtr model)
{
    if (!model_dart_) {
        return;
    }

//    RTT::os::MutexTryLock trylock(gazebo_mutex_);
//    if(!trylock.isSuccessful()) {
//        return;
//    }


//BodyNode::setMomentOfInertia(double _Ixx, double _Iyy, double _Izz, double _Ixy, double _Ixz, double _Iyz)
//BodyNode::setMass(double _mass)
//setLocalCOM

//    dart_sk_->computeInverseDynamics();

    // mass matrix
    const Eigen::MatrixXd &mass_matrix = dart_sk_->getMassMatrix();
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            tmp_r_MassMatrix_out_(i,j) = mass_matrix(r_indices_[i], r_indices_[j]);
            tmp_l_MassMatrix_out_(i,j) = mass_matrix(l_indices_[i], l_indices_[j]);
        }
    }

    // gravity forces
    Eigen::VectorXd grav = dart_sk_->getGravityForces();
    for (int i = 0; i < 7; i++) {
        tmp_r_GravityTorque_out_(i) = grav(r_indices_[i]);
        tmp_l_GravityTorque_out_(i) = grav(l_indices_[i]);
    }

    if (grav.size() != dart_sk_->getNumDofs()) {
        std::cout << "ERROR: VelmaGazebo::gazeboUpdateHook: grav.size() != dart_sk_->getNumDofs()   " << grav.size() << "!=" << dart_sk_->getNumDofs() << std::endl;
        return;
    }

//    std::cout << grav.transpose() << std::endl;

    const Eigen::VectorXd &ext_f = dart_sk_->getExternalForces() + dart_sk_->getConstraintForces() + dart_sk_->getCoriolisForces();
    // external forces
    for (int i = 0; i < 7; i++) {
        tmp_r_JointTorque_out_(i) = ext_f[r_indices_[i]];
        tmp_l_JointTorque_out_(i) = ext_f[l_indices_[i]];
    }

    const double torso_gear = 158.0;
    const double torso_trans_mult = 20000.0 * torso_gear / (M_PI * 2.0);
    const double torso_motor_offset = -88524.0;
    const double torso_joint_offset = 0;
    const double torso_motor_constant = 0.00105;

    // joint position
    for (int i = 0; i < 7; i++) {
        tmp_r_JointPosition_out_(i) = r_joints_[i]->GetAngle(0).Radian();
        tmp_l_JointPosition_out_(i) = l_joints_[i]->GetAngle(0).Radian();
    }
    tmp_t_MotorPosition_out_ = (t_joints_[0]->GetAngle(0).Radian() - torso_joint_offset) * torso_trans_mult + torso_motor_offset;

    // joint velocity
    for (int i = 0; i < 7; i++) {
        tmp_r_JointVelocity_out_(i) = r_joints_[i]->GetVelocity(0);
        tmp_l_JointVelocity_out_(i) = l_joints_[i]->GetVelocity(0);
    }
    tmp_t_MotorVelocity_out_ = t_joints_[0]->GetVelocity(0) * torso_trans_mult;

    gazebo::physics::JointWrench wr_r, wr_l;
    wr_r = r_joints_[6]->GetForceTorque(0);
    wr_l = l_joints_[6]->GetForceTorque(0);
    r_CartesianWrench_out_.force.x = wr_r.body2Force.x;
    r_CartesianWrench_out_.force.y = wr_r.body2Force.y;
    r_CartesianWrench_out_.force.z = wr_r.body2Force.z;
    r_CartesianWrench_out_.torque.x = wr_r.body2Torque.x;
    r_CartesianWrench_out_.torque.y = wr_r.body2Torque.y;
    r_CartesianWrench_out_.torque.z = wr_r.body2Torque.z;

    l_CartesianWrench_out_.force.x = wr_l.body2Force.x;
    l_CartesianWrench_out_.force.y = wr_l.body2Force.y;
    l_CartesianWrench_out_.force.z = wr_l.body2Force.z;
    l_CartesianWrench_out_.torque.x = wr_l.body2Torque.x;
    l_CartesianWrench_out_.torque.y = wr_l.body2Torque.y;
    l_CartesianWrench_out_.torque.z = wr_l.body2Torque.z;

    //
    // head
    //
    const double head_trans = 8000.0 * 100.0 / (M_PI * 2.0);
    hp_q_out_ = -head_pan_joint_->GetAngle(0).Radian() * head_trans;
    ht_q_out_ = head_tilt_joint_->GetAngle(0).Radian() * head_trans;
    hp_v_out_ = head_pan_joint_->GetVelocity(0);
    ht_v_out_ = head_tilt_joint_->GetVelocity(0);

    bool tmp_r_command_mode_;
    bool tmp_l_command_mode_;

    // exchange the data between Orocos and Gazebo
    {
        RTT::os::MutexLock lock(gazebo_mutex_);

        r_MassMatrix_out_ = tmp_r_MassMatrix_out_;
        l_MassMatrix_out_ = tmp_l_MassMatrix_out_;
        r_GravityTorque_out_ = tmp_r_GravityTorque_out_;
        l_GravityTorque_out_ = tmp_l_GravityTorque_out_;
        r_JointTorque_out_ = tmp_r_JointTorque_out_;
        l_JointTorque_out_ = tmp_l_JointTorque_out_;
        r_JointPosition_out_ = tmp_r_JointPosition_out_;
        l_JointPosition_out_ = tmp_l_JointPosition_out_;
        t_MotorPosition_out_ = tmp_t_MotorPosition_out_;
        r_JointVelocity_out_ = tmp_r_JointVelocity_out_;
        l_JointVelocity_out_ = tmp_l_JointVelocity_out_;
        t_MotorVelocity_out_ = tmp_t_MotorVelocity_out_;
        r_CartesianWrench_out_ = tmp_r_CartesianWrench_out_;
        l_CartesianWrench_out_ = tmp_l_CartesianWrench_out_;
        hp_q_out_ = tmp_hp_q_out_;
        ht_q_out_ = tmp_ht_q_out_;
        hp_v_out_ = tmp_hp_v_out_;
        ht_v_out_ = tmp_ht_v_out_;

        tmp_r_JointTorqueCommand_in_ = r_JointTorqueCommand_in_;
        tmp_l_JointTorqueCommand_in_ = l_JointTorqueCommand_in_;
        tmp_t_MotorCurrentCommand_in_ = t_MotorCurrentCommand_in_;
        tmp_hp_q_in_ = hp_q_in_;
        tmp_ht_q_in_ = ht_q_in_;

        tmp_r_command_mode_ = r_command_mode_;
        tmp_l_command_mode_ = l_command_mode_;

        r_JointTorqueCommand_in_.setZero();
        l_JointTorqueCommand_in_.setZero();
        t_MotorCurrentCommand_in_ = 0.0;
    }

    // torque command
    if (tmp_r_command_mode_) {
        for (int i = 0; i < 7; i++) {
            grav(r_indices_[i]) += tmp_r_JointTorqueCommand_in_(i);
//            r_dart_joints_[i]->setForce(0, r_JointTorqueCommand_in_(i) + grav(r_indices_[i]));
//            r_force_prev_(i) = r_JointTorqueCommand_in_(i);
        }
    }

    if (tmp_l_command_mode_) {
        for (int i = 0; i < 7; i++) {
            grav(l_indices_[i]) += tmp_l_JointTorqueCommand_in_(i);
//            l_dart_joints_[i]->setForce(0, l_JointTorqueCommand_in_(i) + grav(l_indices_[i]));
//            l_force_prev_(i) = l_JointTorqueCommand_in_(i);
        }
    }

    grav(t_indices_[0]) += tmp_t_MotorCurrentCommand_in_ * torso_gear * torso_motor_constant;

    dart_sk_->setForces(grav);

//    t_joints_[0]->SetForce(0, tmp_t_MotorCurrentCommand_in_ * torso_gear * torso_motor_constant);

    jc_->SetPositionTarget(head_pan_joint_->GetScopedName(), -tmp_hp_q_in_ / head_trans);
    jc_->SetPositionTarget(head_tilt_joint_->GetScopedName(), tmp_ht_q_in_ / head_trans);

    jc_->Update();
}

