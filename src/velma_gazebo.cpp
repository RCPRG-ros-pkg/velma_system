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

        jc_ = new gazebo::physics::JointController(model);
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
//                "  has pos: " << (rh_joint_dart->hasPositionLimit(0)?"true":"false") <<
                "  en pos: " << (rh_joint_dart->isPositionLimited()?"true":"false") <<
                "  force: " << rh_joint_dart->getForceLowerLimit(0) << " " << rh_joint_dart->getForceUpperLimit(0) << std::endl;

            std::cout << "gz: " << rh_joint->GetName() << "  eff: " << rh_joint->GetEffortLimit(0) <<
                "  vel: " << rh_joint->GetVelocityLimit(0) << std::endl;

            rh_joint->SetEffortLimit(0, 1);

//            rh_joint_dart->setPositionLimited( false);

            gazebo::common::PID pid0(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            gazebo::common::PID pid(1.0, 0.0, 0.0, 0.0, 0.0, 1.0,-1.0);//300.0, -300.0);
            jc_->AddJoint(rh_joint);
//            jc_->SetPositionPID(rh_joint->GetScopedName(), pid);
            jc_->SetVelocityPID(rh_joint->GetScopedName(), pid);

            gazebo::physics::JointPtr lh_joint = model->GetJoint(lh_name);
            lh_joints_.push_back(lh_joint);
            lh_joints_dart_.push_back( dart_sk_->getJoint(lh_name) );
            jc_->AddJoint(lh_joint);
//            jc_->SetPositionPID(lh_joint->GetScopedName(), pid);
            jc_->SetVelocityPID(lh_joint->GetScopedName(), pid);
        }

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

//        std::cout << "pos limits: " << rh_joints_dart_[0]->getPositionLowerLimit(0) << "  " << rh_joints_dart_[0]->getPositionUpperLimit(0) << std::endl;
//        std::cout << "pos: " << rh_joints_dart_[0]->getPosition(0) << std::endl;

        return true;
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

//        r_JointTorque_out_(i) = r_joints_[i]->GetForce(0); //ext_f[r_indices_[i]];
//        l_JointTorque_out_(i) = l_joints_[i]->GetForce(0); //ext_f[l_indices_[i]];
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
//            r_joints_[i]->SetForce(0, r_JointTorqueCommand_in_(i));
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

    jc_->SetVelocityTarget(rh_joints_[0]->GetScopedName(), 1.0);
//    if (!jc_->SetPositionTarget(rh_joints_[3]->GetScopedName(), -3.5)) {
//        std::cout << "joint " << rh_joints_[3]->GetScopedName() << " not found" << std::endl;
//    }
    jc_->SetVelocityTarget(rh_joints_[1]->GetScopedName(), 0);
//    jc_->SetPositionTarget(rh_joints_[1]->GetScopedName(), 0);
    jc_->SetVelocityTarget(rh_joints_[2]->GetScopedName(), 0);
//    jc_->SetPositionTarget(rh_joints_[2]->GetScopedName(), 0);
    jc_->SetVelocityTarget(rh_joints_[3]->GetScopedName(), 0);
//    jc_->SetPositionTarget(rh_joints_[3]->GetScopedName(), 0);
    jc_->SetVelocityTarget(rh_joints_[4]->GetScopedName(), 0);
//    jc_->SetPositionTarget(rh_joints_[4]->GetScopedName(), 0);
    jc_->SetVelocityTarget(rh_joints_[5]->GetScopedName(), 0);
//    jc_->SetPositionTarget(rh_joints_[5]->GetScopedName(), 0);
    jc_->SetVelocityTarget(rh_joints_[6]->GetScopedName(), 0);
//    jc_->SetPositionTarget(rh_joints_[6]->GetScopedName(), 0);
    jc_->SetVelocityTarget(rh_joints_[7]->GetScopedName(), 0);
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

