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

        std::string hand_joint_names[] = {"_HandFingerOneKnuckleOneJoint", "_HandFingerOneKnuckleTwoJoint", "_HandFingerOneKnuckleThreeJoint",
            "_HandFingerTwoKnuckleOneJoint", "_HandFingerTwoKnuckleTwoJoint", "_HandFingerTwoKnuckleThreeJoint",
            "_HandFingerThreeKnuckleTwoJoint", "_HandFingerThreeKnuckleThreeJoint" };

        for (int i = 0; i < 8; i++) {
            std::string rh_name( std::string("right") + hand_joint_names[i] );
            std::string lh_name( std::string("left") + hand_joint_names[i] );
            dart_sk_->getJoint(rh_name)->setActuatorType( dart::dynamics::Joint::FORCE );
            dart_sk_->getJoint(lh_name)->setActuatorType( dart::dynamics::Joint::FORCE );
            gazebo::physics::JointPtr rh_joint = model->GetJoint(rh_name);
            rh_joints_.push_back(rh_joint);
            dart::dynamics::Joint* rh_joint_dart = dart_sk_->getJoint(rh_name);
            rh_joints_dart_.push_back( rh_joint_dart );
            std::cout << rh_joint_dart->getName() << "  acc: " << rh_joint_dart->getAccelerationLowerLimit(0) << " " << rh_joint_dart->getAccelerationUpperLimit(0) <<
                "  pos: " << rh_joint_dart->getPositionLowerLimit(0) << " " << rh_joint_dart->getPositionUpperLimit(0) <<
                "  vel: " << rh_joint_dart->getVelocityLowerLimit(0) << " " << rh_joint_dart->getVelocityUpperLimit(0) <<
                "  en pos: " << (rh_joint_dart->isPositionLimited()?"true":"false") <<
                "  force: " << rh_joint_dart->getForceLowerLimit(0) << " " << rh_joint_dart->getForceUpperLimit(0) << std::endl;

            std::cout << "gz: " << rh_joint->GetName() << "  eff: " << rh_joint->GetEffortLimit(0) <<
                "  vel: " << rh_joint->GetVelocityLimit(0) << std::endl;

            rh_joint->SetEffortLimit(0, 1);

            gazebo::physics::JointPtr lh_joint = model->GetJoint(lh_name);
            lh_joints_.push_back(lh_joint);
            lh_joints_dart_.push_back( dart_sk_->getJoint(lh_name) );
        }

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

        for (int i = 0; i < 3; i++) {
            rh_clutch_break_[i] = false;
            lh_clutch_break_[i] = false;
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

        for (int i = 0; i < 8; i++) {
            jc_->AddJoint(rh_joints_[i]);
            jc_->AddJoint(lh_joints_[i]);
        }
        jc_->SetVelocityPID(rh_joints_[0]->GetScopedName(), gazebo::common::PID(1.0, 0.0, 0.0, 0.0, 0.0, 1.0,-1.0));
        jc_->SetVelocityPID(rh_joints_[3]->GetScopedName(), gazebo::common::PID(1.0, 0.0, 0.0, 0.0, 0.0, 1.0,-1.0));
        jc_->SetVelocityPID(rh_joints_[1]->GetScopedName(), gazebo::common::PID(0.5, 0.0, 0.0, 0.0, 0.0, 0.5,-0.5));
        jc_->SetVelocityPID(rh_joints_[4]->GetScopedName(), gazebo::common::PID(0.5, 0.0, 0.0, 0.0, 0.0, 0.5,-0.5));
        jc_->SetVelocityPID(rh_joints_[6]->GetScopedName(), gazebo::common::PID(0.5, 0.0, 0.0, 0.0, 0.0, 0.5,-0.5));
        jc_->SetVelocityPID(rh_joints_[2]->GetScopedName(), gazebo::common::PID(0.2, 0.0, 0.0, 0.0, 0.0, 0.2,-0.2));
        jc_->SetVelocityPID(rh_joints_[5]->GetScopedName(), gazebo::common::PID(0.2, 0.0, 0.0, 0.0, 0.0, 0.2,-0.2));
        jc_->SetVelocityPID(rh_joints_[7]->GetScopedName(), gazebo::common::PID(0.2, 0.0, 0.0, 0.0, 0.0, 0.2,-0.2));
        jc_->SetVelocityPID(lh_joints_[0]->GetScopedName(), gazebo::common::PID(1.0, 0.0, 0.0, 0.0, 0.0, 1.0,-1.0));
        jc_->SetVelocityPID(lh_joints_[3]->GetScopedName(), gazebo::common::PID(1.0, 0.0, 0.0, 0.0, 0.0, 1.0,-1.0));
        jc_->SetVelocityPID(lh_joints_[1]->GetScopedName(), gazebo::common::PID(0.5, 0.0, 0.0, 0.0, 0.0, 0.5,-0.5));
        jc_->SetVelocityPID(lh_joints_[4]->GetScopedName(), gazebo::common::PID(0.5, 0.0, 0.0, 0.0, 0.0, 0.5,-0.5));
        jc_->SetVelocityPID(lh_joints_[6]->GetScopedName(), gazebo::common::PID(0.5, 0.0, 0.0, 0.0, 0.0, 0.5,-0.5));
        jc_->SetVelocityPID(lh_joints_[2]->GetScopedName(), gazebo::common::PID(0.2, 0.0, 0.0, 0.0, 0.0, 0.2,-0.2));
        jc_->SetVelocityPID(lh_joints_[5]->GetScopedName(), gazebo::common::PID(0.2, 0.0, 0.0, 0.0, 0.0, 0.2,-0.2));
        jc_->SetVelocityPID(lh_joints_[7]->GetScopedName(), gazebo::common::PID(0.2, 0.0, 0.0, 0.0, 0.0, 0.2,-0.2));

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

    //
    // BarrettHand
    //

    const double force_factor = 1000.0;
    // joint position
    for (int i = 0; i < 8; i++) {
        rh_q_out_(i) = rh_joints_[i]->GetAngle(0).Radian();
        lh_q_out_(i) = lh_joints_[i]->GetAngle(0).Radian();
    }

    rh_t_out_[0] = rh_t_out_[3] = rh_joints_[0]->GetForce(0)*force_factor;
    rh_t_out_[1] = rh_t_out_[2] = rh_joints_[1]->GetForce(0)*force_factor;
    rh_t_out_[4] = rh_t_out_[5] = rh_joints_[4]->GetForce(0)*force_factor;
    rh_t_out_[6] = rh_t_out_[7] = rh_joints_[6]->GetForce(0)*force_factor;

    lh_t_out_[0] = lh_t_out_[3] = lh_joints_[0]->GetForce(0)*force_factor;
    lh_t_out_[1] = lh_t_out_[2] = lh_joints_[1]->GetForce(0)*force_factor;
    lh_t_out_[4] = lh_t_out_[5] = lh_joints_[4]->GetForce(0)*force_factor;
    lh_t_out_[6] = lh_t_out_[7] = lh_joints_[6]->GetForce(0)*force_factor;

    // spread joints
    const double vel_trap_angle = 5.0/180.0*3.1415;
    const double spread_dead_angle = 3.0/180.0*3.1415;

    double diff, force;
    int dof_idx, jnt_idx;
    int f1k1_dof_idx = 3;
    int f1k1_jnt_idx = 0;
    int f2k1_dof_idx = 3;
    int f2k1_jnt_idx = 3;

    double f1k1_force = rh_joints_[f1k1_jnt_idx]->GetForce(0);
    double f2k1_force = rh_joints_[f2k1_jnt_idx]->GetForce(0);
    double spread_force = f1k1_force + f2k1_force;

    if (std::fabs(spread_force) > 0.5) {
        rh_status_out_ |= STATUS_OVERCURRENT4;
    }

    double f1k1_angle = rh_joints_[f1k1_jnt_idx]->GetAngle(0).Radian();
    double f2k1_angle = rh_joints_[f2k1_jnt_idx]->GetAngle(0).Radian();

    double f1k1_diff = clip( rh_q_in_[f1k1_dof_idx] - rh_joints_[f1k1_jnt_idx]->GetAngle(0).Radian(), -vel_trap_angle, vel_trap_angle);
    double f2k1_diff = clip( rh_q_in_[f2k1_dof_idx] - rh_joints_[f2k1_jnt_idx]->GetAngle(0).Radian(), -vel_trap_angle, vel_trap_angle);

    double spread_diff = f1k1_angle - f2k1_angle;

    double f1k1_vel = f1k1_diff - spread_diff / 2.0;
    double f2k1_vel = f2k1_diff + spread_diff / 2.0;

    if (std::fabs(f1k1_diff) < vel_trap_angle * 0.2 && std::fabs(f2k1_diff) < vel_trap_angle * 0.2) {
        rh_status_out_ |= STATUS_IDLE4;
    }

    jc_->SetVelocityTarget(rh_joints_[f1k1_jnt_idx]->GetScopedName(), rh_v_in_[f1k1_dof_idx] * f1k1_vel / vel_trap_angle);
    jc_->SetVelocityTarget(rh_joints_[f2k1_jnt_idx]->GetScopedName(), rh_v_in_[f2k1_dof_idx] * f2k1_vel / vel_trap_angle);

    // finger joints
    int k2_dof_tab[3] = {0, 1, 2};
    int k2_jnt_tab[3] = {1, 4, 6};
    int k3_jnt_tab[3] = {2, 5, 7};
    int status_overcurrent_tab[3] = {STATUS_OVERCURRENT1, STATUS_OVERCURRENT2, STATUS_OVERCURRENT3};
    int status_idle_tab[3] = {STATUS_IDLE1, STATUS_IDLE2, STATUS_IDLE3};

    for (int i = 0; i < 3; i++) {
        double k2_diff;
        double k3_diff;

        if (!rh_clutch_break_[i]) {
            k2_diff = clip( rh_q_in_[k2_dof_tab[i]] - rh_joints_[k2_jnt_tab[i]]->GetAngle(0).Radian(), -vel_trap_angle, vel_trap_angle);
            k3_diff = rh_joints_[k2_jnt_tab[i]]->GetAngle(0).Radian() / 3.0 - rh_joints_[k3_jnt_tab[i]]->GetAngle(0).Radian();
            jc_->SetVelocityTarget(rh_joints_[k2_jnt_tab[i]]->GetScopedName(), rh_v_in_[k2_dof_tab[i]] * k2_diff / vel_trap_angle);
            jc_->SetVelocityTarget(rh_joints_[k3_jnt_tab[i]]->GetScopedName(), rh_v_in_[k2_dof_tab[i]] * k3_diff / vel_trap_angle);
        }
        else {
        }

        double k2_force = rh_joints_[k2_jnt_tab[i]]->GetForce(0);
        double k3_force = rh_joints_[k3_jnt_tab[i]]->GetForce(0);
        if (std::fabs(k2_force) > 0.5 || std::fabs(k3_force) > 0.5) {
            rh_status_out_ |= status_overcurrent_tab[i];
        }
        if (std::fabs(k2_diff) < vel_trap_angle * 0.2) {
            rh_status_out_ |= status_idle_tab[i];
        }
    }

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

