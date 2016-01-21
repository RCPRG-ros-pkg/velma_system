#include "velma_gazebo.h"

    bool VelmaGazebo::gazeboConfigureHook(gazebo::physics::ModelPtr model) {

//        RTT::os::MutexLock lock(gazebo_mutex_);

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
/*
        jc_->Reset();
        for (int i = 0; i < 8; i++) {
            gazebo::common::PID pid(1.0, 0.0, 0.0, 0.0, 0.0, 1.0,-1.0);
            jc_->AddJoint(rh_joints_[i]);
            jc_->SetVelocityPID(rh_joints_[i]->GetScopedName(), pid);
            jc_->AddJoint(lh_joints_[i]);
            jc_->SetVelocityPID(lh_joints_[i]->GetScopedName(), pid);
        }
*/

//        std::cout << rh_joints_[0]->GetScopedName() << std::endl;

//        std::map< std::string,gazebo::common::PID > map = jc_->GetVelocityPIDs();
//        for (std::map< std::string,gazebo::common::PID >::const_iterator it = map.begin(); it != map.end(); it++) {
//            std::cout << "PID map: " << it->first << std::endl;
//        }


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
            jc_->AddJoint(joint);
            jc_->SetPositionPID(joint->GetScopedName(), gazebo::common::PID(0.8, 0.0, 0.5, 0.2, -0.2, 1.0,-1.0));

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
            jc_->AddJoint(joint);
            jc_->SetPositionPID(joint->GetScopedName(), gazebo::common::PID(0.8, 0.0, 0.5, 0.2, -0.2, 1.0,-1.0));
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

        for (int bidx = 0; bidx < dart_sk_->getNumBodyNodes(); bidx++) {
            dart::dynamics::BodyNode *b = dart_sk_->getBodyNode(bidx);
            std::cout << "BodyNode " << bidx << "  " << b->getName() << std::endl;
        }

        for (gazebo::physics::Link_V::const_iterator it = model->GetLinks().begin(); it != model->GetLinks().end(); it++) {
            std::cout << "link: " << (*it)->GetName() << std::endl;
        }

        for (int i = 0; i < 3; i++) {
            rh_clutch_break_[i] = false;
            lh_clutch_break_[i] = false;
        }

        double angle = 90.0/180.0*3.1415;
        jc_->SetPositionTarget(r_joints_[0]->GetScopedName(), -angle/3.0);
        jc_->SetPositionTarget(l_joints_[0]->GetScopedName(), angle/3.0);

        jc_->SetPositionTarget(r_joints_[1]->GetScopedName(), -angle);
        jc_->SetPositionTarget(l_joints_[1]->GetScopedName(), angle);

        jc_->SetPositionTarget(r_joints_[2]->GetScopedName(), angle);
        jc_->SetPositionTarget(l_joints_[2]->GetScopedName(), -angle);

        jc_->SetPositionTarget(r_joints_[3]->GetScopedName(), angle);
        jc_->SetPositionTarget(l_joints_[3]->GetScopedName(), -angle);

        jc_->SetPositionTarget(r_joints_[5]->GetScopedName(), -angle);
        jc_->SetPositionTarget(l_joints_[5]->GetScopedName(), angle);

        move_to_init_pose_ = true;

//        std::cout << "pos limits: " << rh_joints_dart_[0]->getPositionLowerLimit(0) << "  " << rh_joints_dart_[0]->getPositionUpperLimit(0) << std::endl;
//        std::cout << "pos: " << rh_joints_dart_[0]->getPosition(0) << std::endl;

        return true;
    }

double VelmaGazebo::clip(double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void VelmaGazebo::gazeboUpdateHook(gazebo::physics::ModelPtr model)
{
//    std::cout << "gazeboUpdateHook" << std::endl;
    if (!model_dart_) {
        return;
    }

    RTT::os::MutexTryLock trylock(gazebo_mutex_);
    if(!trylock.isSuccessful()) {
        return;
    }

    if (move_to_init_pose_) {
        if (gazebo::physics::get_world()->GetSimTime().Double() > 15) {
            jc_->Reset();
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
            move_to_init_pose_ = false;
        }
        else {
            jc_->Update();
            return;
        }
    }



/*
    if (init_pos_counter_ == 0) {
        double angle = 100.0/180.0*3.1415;
        bool satisfied = true;
        if (r_joints_[1]->GetAngle(0).Radian() > -angle) {
            r_joints_[1]->SetForce(0, -0.5);
            satisfied = false;
        }
        if (l_joints_[1]->GetAngle(0).Radian() < angle) {
            l_joints_[1]->SetForce(0, 0.5);
            satisfied = false;
        }
        if (satisfied) {
            init_pos_counter_++;
        }
    }
    else if (init_pos_counter_ == 1) {
        double angle = 100.0/180.0*3.1415;
        bool satisfied = true;
        if (r_joints_[2]->GetAngle(0).Radian() < angle) {
            r_joints_[2]->SetForce(0, 0.5);
            satisfied = false;
        }
        if (l_joints_[2]->GetAngle(0).Radian() > -angle) {
            l_joints_[2]->SetForce(0, -0.5);
            satisfied = false;
        }
        if (satisfied) {
            init_pos_counter_++;
        }
    }
    else if (init_pos_counter_ == 2) {
        double angle = 100.0/180.0*3.1415;
        bool satisfied = true;
        if (r_joints_[3]->GetAngle(0).Radian() < angle) {
            r_joints_[3]->SetForce(0, 0.5);
            satisfied = false;
        }
        if (l_joints_[3]->GetAngle(0).Radian() > -angle) {
            l_joints_[3]->SetForce(0, -0.5);
            satisfied = false;
        }
        if (satisfied) {
            init_pos_counter_++;
        }
    }
*/

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
    counts_++;
//    std::cout << "c1: " << counts_ << std::endl;

    counts2_ = 0;
/*
    dart::dynamics::BodyNode *bn = dart_sk_->getBodyNode("right_HandPalmLink");
    if (bn == NULL) {
        std::cout << "BodyNode is NULL" << std::endl;
    }
    else {
        int num_sh = bn->getNumCollisionShapes();
        for (int i = 0; i < num_sh; i++) {
            dart::dynamics::Shape *sh = dart_sk_->getBodyNode("right_HandPalmLink")->getCollisionShape(i);
            if (sh == NULL) {
                std::cout << "shape " << i << "  is NULL" << std::endl;
                continue;
            }
            if (sh->getShapeType() == dart::dynamics::Shape::MESH) {
                dart::dynamics::MeshShape *msh = static_cast<dart::dynamics::MeshShape*>(sh);
                const aiScene *sc = msh->getMesh();
                std::cout << "shape " << i << "  vertices: " << sc->mMeshes[0]->mNumVertices << std::endl;
                for (int vidx = 0; vidx < sc->mMeshes[0]->mNumVertices; vidx++) {
    //                sc->mMeshes[0]->mVertices[vidx].x
                }
            }
        }
    }
//*/
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
    //const Eigen::VectorXd &ext_f = dart_sk_->getExternalForces();
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
        r_JointTorque_out_(i) = r_joints_[i]->GetForce(0); //ext_f[r_indices_[i]];
        l_JointTorque_out_(i) = l_joints_[i]->GetForce(0); //ext_f[l_indices_[i]];
    }

    // joint position
    for (int i = 0; i < 7; i++) {
        r_JointPosition_out_(i) = r_joints_[i]->GetAngle(0).Radian();
        l_JointPosition_out_(i) = l_joints_[i]->GetAngle(0).Radian();
    }
    for (int i = 0; i < t_joints_.size(); i++) {
        t_JointPosition_out_(i) = t_joints_[i]->GetAngle(0).Radian();
    }

    // joint velocity
    for (int i = 0; i < 7; i++) {
        r_JointVelocity_out_(i) = r_joints_[i]->GetVelocity(0);
        l_JointVelocity_out_(i) = l_joints_[i]->GetVelocity(0);
    }
    for (int i = 0; i < t_joints_.size(); i++) {
        t_JointVelocity_out_(i) = t_joints_[i]->GetVelocity(0);
    }

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

    for (int i = 0; i < t_joints_.size(); i++) {
        t_joints_[i]->SetForce(0, t_JointTorqueCommand_in_(i));
    }
    t_JointTorqueCommand_in_.setZero();

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


//    rh_joints_dart_[0]->setForce(0, 5.0);
//    rh_joints_dart_[1]->setCommand(0, 10.0);
//    rh_joints_dart_[0]->setCommand(0, 0.2);
//    std::cout << force << std::endl;
/*    if (force > rh_t_in_[dof_idx]/force_factor || force < -rh_t_in_[dof_idx]/force_factor) {
        if (!jc_->SetVelocityTarget(rh_joints_[jnt_idx]->GetScopedName(), 0)) {
            std::cout << "joint " << rh_joints_[jnt_idx]->GetScopedName() << " not found" << std::endl;
        }
        jc_->SetPositionTarget(rh_joints_[jnt_idx]->GetScopedName(), rh_joints_[jnt_idx]->GetAngle(0).Radian());
//        rh_joints_dart_[jnt_idx]->setCommand(0, 0);
    }
    else {
        if (!jc_->SetVelocityTarget(rh_joints_[jnt_idx]->GetScopedName(), rh_v_in_[dof_idx] * diff / vel_trap_angle)) {
            std::cout << "joint " << rh_joints_[jnt_idx]->GetScopedName() << " not found" << std::endl;
        }
        jc_->SetPositionTarget(rh_joints_[jnt_idx]->GetScopedName(), rh_q_in_[dof_idx]);
//        rh_joints_dart_[jnt_idx]->setCommand(0, rh_v_in_[dof_idx] * diff / vel_trap_angle);
    }
*/

//    jc_->SetVelocityTarget(rh_joints_[0]->GetScopedName(), 1.0);
//    if (!jc_->SetPositionTarget(rh_joints_[3]->GetScopedName(), -3.5)) {
//        std::cout << "joint " << rh_joints_[3]->GetScopedName() << " not found" << std::endl;
//    }
//    jc_->SetVelocityTarget(rh_joints_[1]->GetScopedName(), 0);
//    jc_->SetPositionTarget(rh_joints_[1]->GetScopedName(), 0);
//    jc_->SetVelocityTarget(rh_joints_[2]->GetScopedName(), 0);
//    jc_->SetPositionTarget(rh_joints_[2]->GetScopedName(), 0);
//    jc_->SetVelocityTarget(rh_joints_[3]->GetScopedName(), 0);
//    jc_->SetPositionTarget(rh_joints_[3]->GetScopedName(), 0);
//    jc_->SetVelocityTarget(rh_joints_[4]->GetScopedName(), 0);
//    jc_->SetPositionTarget(rh_joints_[4]->GetScopedName(), 0);
//    jc_->SetVelocityTarget(rh_joints_[5]->GetScopedName(), 0);
//    jc_->SetPositionTarget(rh_joints_[5]->GetScopedName(), 0);
//    jc_->SetVelocityTarget(rh_joints_[6]->GetScopedName(), 0);
//    jc_->SetPositionTarget(rh_joints_[6]->GetScopedName(), 0);
//    jc_->SetVelocityTarget(rh_joints_[7]->GetScopedName(), 0);
//    jc_->SetPositionTarget(rh_joints_[7]->GetScopedName(), 0);
    jc_->Update();

/*
    double diff, force;
    int dof_idx, jnt_idx;
    dof_idx = 3;
    jnt_idx = 0;
    diff = rh_q_in_[dof_idx] - rh_joints_[jnt_idx]->GetAngle(0).Radian();
    if (diff > vel_trap_angle) {
        diff = vel_trap_angle;
    }
    if (diff < -vel_trap_angle) {
        diff = -vel_trap_angle;
    }
    force = rh_joints_dart_[jnt_idx]->getForce(0);
    if (force > rh_t_in_[dof_idx]/force_factor || force < -rh_t_in_[dof_idx]/force_factor) {
        rh_joints_dart_[jnt_idx]->setCommand(0, 0);
    }
    else {
        rh_joints_dart_[jnt_idx]->setCommand(0, rh_v_in_[dof_idx] * diff / vel_trap_angle);
    }

    dof_idx = 3;
    jnt_idx = 3;
    diff = rh_q_in_[dof_idx] - rh_joints_[jnt_idx]->GetAngle(0).Radian();
    if (diff > vel_trap_angle) {
        diff = vel_trap_angle;
    }
    if (diff < -vel_trap_angle) {
        diff = -vel_trap_angle;
    }
    force = rh_joints_dart_[jnt_idx]->getForce(0);
    if (force > rh_t_in_[dof_idx]/force_factor || force < -rh_t_in_[dof_idx]/force_factor) {
        rh_joints_dart_[jnt_idx]->setCommand(0, 0);
    }
    else {
        rh_joints_dart_[jnt_idx]->setCommand(0, rh_v_in_[dof_idx] * diff / vel_trap_angle);
    }

    // f1 joints
    dof_idx = 0;
    jnt_idx = 1;
    diff = rh_q_in_[dof_idx] - rh_joints_[jnt_idx]->GetAngle(0).Radian();
    if (diff > vel_trap_angle) {
        diff = vel_trap_angle;
    }
    if (diff < -vel_trap_angle) {
        diff = -vel_trap_angle;
    }
    force = rh_joints_dart_[jnt_idx]->getForce(0);
    if (force > rh_t_in_[dof_idx]/force_factor || force < -rh_t_in_[dof_idx]/force_factor) {
        rh_joints_dart_[jnt_idx]->setCommand(0, 0);
    }
    else {
        rh_joints_dart_[jnt_idx]->setCommand(0, rh_v_in_[dof_idx] * diff / vel_trap_angle);
    }

    dof_idx = 0;
    jnt_idx = 2;
    diff = rh_q_in_[dof_idx]*0.3333333 - rh_joints_[jnt_idx]->GetAngle(0).Radian();
    if (diff > vel_trap_angle) {
        diff = vel_trap_angle;
    }
    if (diff < -vel_trap_angle) {
        diff = -vel_trap_angle;
    }
    force = rh_joints_dart_[jnt_idx]->getForce(0);
    if (force > rh_t_in_[dof_idx]/force_factor || force < -rh_t_in_[dof_idx]/force_factor) {
        rh_joints_dart_[jnt_idx]->setCommand(0, 0);
    }
    else {
        rh_joints_dart_[jnt_idx]->setCommand(0, rh_v_in_[dof_idx] * diff / vel_trap_angle);
    }

    // f2 joints
    dof_idx = 1;
    jnt_idx = 4;
    diff = rh_q_in_[dof_idx] - rh_joints_[jnt_idx]->GetAngle(0).Radian();
    if (diff > vel_trap_angle) {
        diff = vel_trap_angle;
    }
    if (diff < -vel_trap_angle) {
        diff = -vel_trap_angle;
    }
    force = rh_joints_dart_[jnt_idx]->getForce(0);
    if (force > rh_t_in_[dof_idx]/force_factor || force < -rh_t_in_[dof_idx]/force_factor) {
        rh_joints_dart_[jnt_idx]->setCommand(0, 0);
    }
    else {
        rh_joints_dart_[jnt_idx]->setCommand(0, rh_v_in_[dof_idx] * diff / vel_trap_angle);
    }

    dof_idx = 1;
    jnt_idx = 5;
    diff = rh_q_in_[dof_idx]*0.3333333 - rh_joints_[jnt_idx]->GetAngle(0).Radian();
    if (diff > vel_trap_angle) {
        diff = vel_trap_angle;
    }
    if (diff < -vel_trap_angle) {
        diff = -vel_trap_angle;
    }
    force = rh_joints_dart_[jnt_idx]->getForce(0);
    if (force > rh_t_in_[dof_idx]/force_factor || force < -rh_t_in_[dof_idx]/force_factor) {
        rh_joints_dart_[jnt_idx]->setCommand(0, 0);
    }
    else {
        rh_joints_dart_[jnt_idx]->setCommand(0, rh_v_in_[dof_idx] * diff / vel_trap_angle);
    }

    // f3 joints
    dof_idx = 2;
    jnt_idx = 6;
    diff = rh_q_in_[dof_idx] - rh_joints_[jnt_idx]->GetAngle(0).Radian();
    if (diff > vel_trap_angle) {
        diff = vel_trap_angle;
    }
    if (diff < -vel_trap_angle) {
        diff = -vel_trap_angle;
    }
    force = rh_joints_dart_[jnt_idx]->getForce(0);
    if (force > rh_t_in_[dof_idx]/force_factor || force < -rh_t_in_[dof_idx]/force_factor) {
        rh_joints_dart_[jnt_idx]->setCommand(0, 0);
    }
    else {
        rh_joints_dart_[jnt_idx]->setCommand(0, rh_v_in_[dof_idx] * diff / vel_trap_angle);
    }

    dof_idx = 2;
    jnt_idx = 7;
    diff = rh_q_in_[dof_idx]*0.3333333 - rh_joints_[jnt_idx]->GetAngle(0).Radian();
    if (diff > vel_trap_angle) {
        diff = vel_trap_angle;
    }
    if (diff < -vel_trap_angle) {
        diff = -vel_trap_angle;
    }
    force = rh_joints_dart_[jnt_idx]->getForce(0);
    if (force > rh_t_in_[dof_idx]/force_factor || force < -rh_t_in_[dof_idx]/force_factor) {
        rh_joints_dart_[jnt_idx]->setCommand(0, 0);
    }
    else {
        rh_joints_dart_[jnt_idx]->setCommand(0, rh_v_in_[dof_idx] * diff / vel_trap_angle);
    }
*/
// TODO:
//    geometry_msgs::Wrench r_CartesianWrench_out_;

}

