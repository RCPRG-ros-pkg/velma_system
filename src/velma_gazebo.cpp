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
        for(unsigned int i = 0; i < 7; i++) {
            std::string joint_name = std::string("right_arm_") + std::to_string(i) + "_joint";
            gazebo::physics::JointPtr joint = model->GetJoint(joint_name);
            gazebo::physics::DARTJointPtr joint_dart = boost::dynamic_pointer_cast < gazebo::physics::DARTJoint > ( joint );
            if (joint) {
                this->r_joints_.push_back(joint);
                r_indices_.push_back(joint_dart->GetDARTJoint()->getIndexInSkeleton(0));
            }
            else {
                RTT::log(RTT::Error) << "A joint named " << joint_name.c_str() << " is not part of Mechanism Controlled joints." << RTT::endlog();
                return false;
            }
        }

        for(unsigned int i = 0; i < 7; i++) {
            std::string joint_name = std::string("left_arm_") + std::to_string(i) + "_joint";
            gazebo::physics::JointPtr joint = model->GetJoint(joint_name);
            gazebo::physics::DARTJointPtr joint_dart = boost::dynamic_pointer_cast < gazebo::physics::DARTJoint > ( joint );
            if (joint) {
                this->l_joints_.push_back(joint);
                l_indices_.push_back(joint_dart->GetDARTJoint()->getIndexInSkeleton(0));
            }
            else {
                RTT::log(RTT::Error) << "A joint named " << joint_name.c_str() << " is not part of Mechanism Controlled joints." << RTT::endlog();
                return false;
            }
        }

        for(unsigned int i = 0; i < 1; i++) {
            std::string joint_name = std::string("torso_") + std::to_string(i) + "_joint";
            gazebo::physics::JointPtr joint = model->GetJoint(joint_name);     
            if (joint) {
                this->t_joints_.push_back(joint);
            }
            else {
                RTT::log(RTT::Error) << "A joint named " << joint_name.c_str() << " is not part of Mechanism Controlled joints." << RTT::endlog();
                return false;
            }
        }

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

        double angle = 90.0/180.0*3.1415;

        boost::dynamic_pointer_cast < gazebo::physics::DARTJoint > ( r_joints_[1] )->GetDARTJoint()->setPosition(0, -angle);
        boost::dynamic_pointer_cast < gazebo::physics::DARTJoint > ( l_joints_[1] )->GetDARTJoint()->setPosition(0, angle);

        boost::dynamic_pointer_cast < gazebo::physics::DARTJoint > ( r_joints_[2] )->GetDARTJoint()->setPosition(0, angle);
        boost::dynamic_pointer_cast < gazebo::physics::DARTJoint > ( l_joints_[2] )->GetDARTJoint()->setPosition(0, -angle);

        boost::dynamic_pointer_cast < gazebo::physics::DARTJoint > ( r_joints_[3] )->GetDARTJoint()->setPosition(0, angle);
        boost::dynamic_pointer_cast < gazebo::physics::DARTJoint > ( l_joints_[3] )->GetDARTJoint()->setPosition(0, -angle);

        boost::dynamic_pointer_cast < gazebo::physics::DARTJoint > ( r_joints_[5] )->GetDARTJoint()->setPosition(0, -angle);
        boost::dynamic_pointer_cast < gazebo::physics::DARTJoint > ( l_joints_[5] )->GetDARTJoint()->setPosition(0, angle);

        head_pan_joint_ = model->GetJoint("head_pan_joint");
        head_tilt_joint_ = model->GetJoint("head_tilt_joint");

        jc_->AddJoint(head_pan_joint_);
        jc_->SetPositionPID(head_pan_joint_->GetScopedName(), gazebo::common::PID(1.0, 0.5, 0.0, 0.1, -0.1, 1.0,-1.0));

        jc_->AddJoint(head_tilt_joint_);
        jc_->SetPositionPID(head_tilt_joint_->GetScopedName(), gazebo::common::PID(1.0, 0.5, 0.0, 0.1, -0.1, 1.0,-1.0));

        return true;
    }

double VelmaGazebo::clip(double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void VelmaGazebo::gazeboUpdateHook(gazebo::physics::ModelPtr model)
{
    if (!model_dart_) {
        return;
    }

    RTT::os::MutexTryLock trylock(gazebo_mutex_);
    if(!trylock.isSuccessful()) {
        return;
    }

/*
    dart::collision::CollisionDetector* detector = dart_world_->getConstraintSolver()->getCollisionDetector();
    detector->detectCollision(true, true);
    size_t collisionCount = detector->getNumContacts();
    if (collisionCount > 0) {
        std::cout << "col: " << collisionCount << std::endl;
        for(size_t i = 0; i < collisionCount; ++i)
        {
          const dart::collision::Contact& contact = detector->getContact(i);
            std::cout << contact.bodyNode1->getName() << " " << contact.bodyNode2->getName() << std::endl;
//          if(contact.bodyNode1.lock()->getSkeleton() == object
//             || contact.bodyNode2.lock()->getSkeleton() == object)
//          {
//            collision = true;
//            break;
//          }
        }
    }
*/
    // mass matrix
    const Eigen::MatrixXd &mass_matrix = dart_sk_->getMassMatrix();
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            r_MassMatrix_out_(i,j) = mass_matrix(r_indices_[i], r_indices_[j]);
            l_MassMatrix_out_(i,j) = mass_matrix(l_indices_[i], l_indices_[j]);
        }
    }

    // gravity forces
    const Eigen::VectorXd &grav = dart_sk_->getGravityForces();
    for (int i = 0; i < 7; i++) {
        r_GravityTorque_out_(i) = grav[r_indices_[i]];
        l_GravityTorque_out_(i) = grav[l_indices_[i]];
    }

    // external forces
    for (int i = 0; i < 7; i++) {
/*
        gazebo::physics::JointWrench r_wr, l_wr;
        r_wr = r_joints_[i]->GetForceTorque(0);
        l_wr = l_joints_[i]->GetForceTorque(0);
        if (i%2 == 0) {
            r_JointTorque_out_(i) = r_wr.body2Torque.z;
            l_JointTorque_out_(i) = l_wr.body2Torque.z;
        }
        else {
            r_JointTorque_out_(i) = r_wr.body2Torque.y;
            l_JointTorque_out_(i) = l_wr.body2Torque.y;
        }
*/
        r_JointTorque_out_(i) = r_joints_[i]->GetForce(0);
        l_JointTorque_out_(i) = l_joints_[i]->GetForce(0);
    }

    const double torso_gear = 158.0;
    const double torso_trans_mult = 20000.0 * torso_gear / (M_PI * 2.0);
    const double torso_motor_offset = -88524.0;
    const double torso_joint_offset = 0;
    const double torso_motor_constant = 0.00105;

    // joint position
    for (int i = 0; i < 7; i++) {
        r_JointPosition_out_(i) = r_joints_[i]->GetAngle(0).Radian();
        l_JointPosition_out_(i) = l_joints_[i]->GetAngle(0).Radian();
    }
    t_MotorPosition_out_ = (t_joints_[0]->GetAngle(0).Radian() - torso_joint_offset) * torso_trans_mult + torso_motor_offset;

    // joint velocity
    for (int i = 0; i < 7; i++) {
        r_JointVelocity_out_(i) = r_joints_[i]->GetVelocity(0);
        l_JointVelocity_out_(i) = l_joints_[i]->GetVelocity(0);
    }
    t_MotorVelocity_out_ = t_joints_[0]->GetVelocity(0) * torso_trans_mult;

    // torque command
    if (r_command_mode_) {
        for (int i = 0; i < 7; i++) {
            r_joints_[i]->SetForce(0, r_JointTorqueCommand_in_(i));
        }
        r_JointTorqueCommand_in_.setZero();
    }

    if (l_command_mode_) {
        for (int i = 0; i < 7; i++) {
            l_joints_[i]->SetForce(0, l_JointTorqueCommand_in_(i));
        }
        l_JointTorqueCommand_in_.setZero();
    }

    t_joints_[0]->SetForce(0, t_MotorCurrentCommand_in_ * torso_gear * torso_motor_constant);
    t_MotorCurrentCommand_in_ = 0.0;

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
    jc_->SetPositionTarget(head_pan_joint_->GetScopedName(), -hp_q_in_ / head_trans);
    jc_->SetPositionTarget(head_tilt_joint_->GetScopedName(), ht_q_in_ / head_trans);

    jc_->Update();

// TODO:
//    geometry_msgs::Wrench r_CartesianWrench_out_;

}

