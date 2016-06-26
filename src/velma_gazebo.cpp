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
#include "rtt_rosclock/rtt_rosclock.h"
#include <gazebo/physics/ode/ODELink.hh>
#include <gazebo/physics/ode/ODECollision.hh>
#include <gazebo/physics/ode/ODEPhysics.hh>

void VelmaGazebo::getMassJointPositions(Eigen::VectorXd &q) {
    std::string joint_names[15] = {"torso_0_joint", "right_arm_0_joint", "right_arm_1_joint",
        "right_arm_2_joint", "right_arm_3_joint", "right_arm_4_joint", "right_arm_5_joint",
        "right_arm_6_joint", "left_arm_0_joint", "left_arm_1_joint", "left_arm_2_joint",
        "left_arm_3_joint", "left_arm_4_joint", "left_arm_5_joint", "left_arm_6_joint"};

    for (int i=0; i<15; i++) {
        q(i) = model_->GetJoint(joint_names[i])->GetAngle(0).Radian();
    }
}

void VelmaGazebo::getExternalForces(Eigen::VectorXd &q) {
    std::string joint_names[14] = {"right_arm_0_joint", "right_arm_1_joint",
        "right_arm_2_joint", "right_arm_3_joint", "right_arm_4_joint", "right_arm_5_joint",
        "right_arm_6_joint", "left_arm_0_joint", "left_arm_1_joint", "left_arm_2_joint",
        "left_arm_3_joint", "left_arm_4_joint", "left_arm_5_joint", "left_arm_6_joint"};

    for (int i=0; i<14; i++) {
        q(i) = model_->GetJoint(joint_names[i])->GetForce(0);
    }
}

void VelmaGazebo::getJointPositionAndVelocity(Eigen::VectorXd &q, Eigen::VectorXd &dq) {
    std::string joint_names[15] = {"torso_0_joint", "right_arm_0_joint", "right_arm_1_joint",
        "right_arm_2_joint", "right_arm_3_joint", "right_arm_4_joint", "right_arm_5_joint",
        "right_arm_6_joint", "left_arm_0_joint", "left_arm_1_joint", "left_arm_2_joint",
        "left_arm_3_joint", "left_arm_4_joint", "left_arm_5_joint", "left_arm_6_joint"};

    for (int i=0; i<15; i++) {
        q(i) = model_->GetJoint(joint_names[i])->GetAngle(0).Radian();
        dq(i) = model_->GetJoint(joint_names[i])->GetVelocity(0);
    }
}

void VelmaGazebo::getHeadJointPositionAndVelocity(Eigen::VectorXd &q, Eigen::VectorXd &dq) {
    std::string joint_names[2] = {"head_pan_joint", "head_tilt_joint"};

    for (int i=0; i<2; i++) {
        q(i) = model_->GetJoint(joint_names[i])->GetAngle(0).Radian();
        dq(i) = model_->GetJoint(joint_names[i])->GetVelocity(0);
    }
}

void VelmaGazebo::setForces(const Eigen::VectorXd &t) {
    std::string joint_names[15] = {"torso_0_joint", "right_arm_0_joint", "right_arm_1_joint",
        "right_arm_2_joint", "right_arm_3_joint", "right_arm_4_joint", "right_arm_5_joint",
        "right_arm_6_joint", "left_arm_0_joint", "left_arm_1_joint", "left_arm_2_joint",
        "left_arm_3_joint", "left_arm_4_joint", "left_arm_5_joint", "left_arm_6_joint"};

    for (int i=0; i<15; i++) {
        model_->GetJoint(joint_names[i])->SetForce(0, t(i));
    }
}

KDL::Vector gz2kdl(const gazebo::math::Vector3 &v) {
    return KDL::Vector(v.x, v.y, v.z);
}

KDL::Frame gz2kdl(const gazebo::math::Pose &p) {
    return KDL::Frame(KDL::Rotation::Quaternion(p.rot.x,p.rot.y,p.rot.z,p.rot.w), gz2kdl(p.pos));
}

void VelmaGazebo::getGravComp(Eigen::VectorXd &t) {
//    std::cout << "getGravComp" << std::endl;

    t(0) = 0.0;

//    std::cout << model_->GetJoint("right_arm_6_joint")->GetJointLink(0)->GetName() << std::endl;

    KDL::Vector gr = gz2kdl( gazebo::physics::get_world()->GetPhysicsEngine()->GetGravity() );

    gazebo::physics::LinkPtr link = model_->GetLink("right_arm_7_link");
    gazebo::physics::JointPtr joint = model_->GetJoint("right_arm_6_joint");
    KDL::Frame T_W_L7 = gz2kdl( link->GetWorldPose() );
    KDL::Vector cog = T_W_L7 * KDL::Vector(r_tool_x_, r_tool_y_, r_tool_z_);
    KDL::Vector r = cog-gz2kdl( joint->GetWorldPose().pos );
    double mass = -r_tool_weight_ / gr.z();
    KDL::Vector torque = r * (mass * gr);
    KDL::Vector axis = gz2kdl( joint->GetGlobalAxis(0) );
    t(7) = KDL::dot(axis, torque);

    for (int i=6; i>0; i--) {
        link = model_->GetLink(std::string("right_arm_") + std::to_string(i) + "_link");
        joint = model_->GetJoint(std::string("right_arm_") + std::to_string(i-1) + "_joint");
        cog = (cog*mass + gz2kdl(link->GetWorldCoGPose().pos)*link->GetInertial()->GetMass()) / (mass+link->GetInertial()->GetMass());
        mass += link->GetInertial()->GetMass();
        r = cog-gz2kdl( joint->GetWorldPose().pos );
        torque = r * (mass * gr);
        axis = gz2kdl( joint->GetGlobalAxis(0) );
        t(i-1+1) = KDL::dot(axis, torque);
    }

    link = model_->GetLink("left_arm_7_link");
    joint = model_->GetJoint("left_arm_6_joint");
    T_W_L7 = gz2kdl( link->GetWorldPose() );
    cog = T_W_L7 * KDL::Vector(l_tool_x_, l_tool_y_, l_tool_z_);
    r = cog-gz2kdl( joint->GetWorldPose().pos );
    mass = -l_tool_weight_ / gr.z();
    torque = r * (mass * gr);
    axis = gz2kdl( joint->GetGlobalAxis(0) );
    t(14) = KDL::dot(axis, torque);

    for (int i=6; i>0; i--) {
        link = model_->GetLink(std::string("left_arm_") + std::to_string(i) + "_link");
        joint = model_->GetJoint(std::string("left_arm_") + std::to_string(i-1) + "_joint");
        cog = (cog*mass + gz2kdl(link->GetWorldCoGPose().pos)*link->GetInertial()->GetMass()) / (mass+link->GetInertial()->GetMass());
        mass += link->GetInertial()->GetMass();
        r = cog-gz2kdl( joint->GetWorldPose().pos );
        torque = r * (mass * gr);
        axis = gz2kdl( joint->GetGlobalAxis(0) );
        t(i-1+8) = KDL::dot(axis, torque);
    }

    t = -t;
}

bool VelmaGazebo::gazeboConfigureHook(gazebo::physics::ModelPtr model) {

        if(model.get() == NULL) {
            std::cout << "VelmaGazebo::gazeboConfigureHook: the gazebo model is NULL" << std::endl;
            return false;
        }

        model_ = model;

        mass_q_.resize(15);


/*
        dart_world_ = boost::dynamic_pointer_cast < gazebo::physics::DARTPhysics > ( gazebo::physics::get_world()->GetPhysicsEngine() ) -> GetDARTWorld();

        model_dart_ = boost::dynamic_pointer_cast < gazebo::physics::DARTModel >(model);
        if (model_dart_.get() == NULL) {
            std::cout << "VelmaGazebo::gazeboConfigureHook: the gazebo model is not a DART model" << std::endl;
            return false;
        }

        dart_sk_ = model_dart_->GetDARTSkeleton();
*/
        jc_ = new gazebo::physics::JointController(model);

        // head joints
        head_pan_joint_ = model->GetJoint("head_pan_joint");
        head_tilt_joint_ = model->GetJoint("head_tilt_joint");

    Eigen::VectorXd grav(15);
    getGravComp(grav);

/*
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

    // data needed for wrist wrench estimation
    right_tool_mass_ = 0.0;
    right_tool_bn_dart_ = dart_sk_->getBodyNode("right_arm_7_link");
    Eigen::Isometry3d ET_WO_Wr = right_tool_bn_dart_->getTransform();
    Eigen::Vector3d comr(0,0,0);
    for (int i = 0; i < dart_sk_->getNumBodyNodes(); i++) {
        dart::dynamics::BodyNode *bn = dart_sk_->getBodyNode(i);
        if (bn->getName().find("right_Hand") == 0 || bn->getName().find("right_arm_7_link") == 0) {
            right_tool_mass_ += bn->getMass();
            Eigen::Isometry3d ET_WO_L = bn->getTransform();
            Eigen::Isometry3d ET_W_L = ET_WO_Wr.inverse() * ET_WO_L;
            comr += ET_W_L * bn->getLocalCOM() * bn->getMass();            
        }
    }
    comr /= right_tool_mass_;
    right_tool_com_W_ = KDL::Vector(comr(0), comr(1), comr(2));

//boost::dynamic_pointer_cast < gazebo::physics::DARTJoint > ( model_->GetJoint(joint_name_) )->GetDARTJoint()->getChildBodyNode();

    left_tool_mass_ = 0.0;
    left_tool_bn_dart_ = dart_sk_->getBodyNode("left_arm_7_link");
    Eigen::Isometry3d ET_WO_Wl = left_tool_bn_dart_->getTransform();
    Eigen::Vector3d coml(0,0,0);
    for (int i = 0; i < dart_sk_->getNumBodyNodes(); i++) {
        dart::dynamics::BodyNode *bn = dart_sk_->getBodyNode(i);
        if (bn->getName().find("left_Hand") == 0 || bn->getName().find("left_arm_7_link") == 0) {
            left_tool_mass_ += bn->getMass();
            Eigen::Isometry3d ET_WO_L = bn->getTransform();
            Eigen::Isometry3d ET_W_L = ET_WO_Wl.inverse() * ET_WO_L;
            coml += ET_W_L * bn->getLocalCOM() * bn->getMass();            
        }
    }
    coml /= left_tool_mass_;
    left_tool_com_W_ = KDL::Vector(coml(0), coml(1), coml(2));

//    std::cout << "mass: " << mass << "  COM: " << com.transpose() << std::endl;


//BodyNode::setMass(double _mass)
//setLocalCOM

    r_force_prev_.resize(7);
    l_force_prev_.resize(7);

    setInitialPosition();

//    setJointsDisabledPID();
    setJointsEnabledPID();

*/
    rtt_rosclock::set_sim_clock_activity(this);

    return true;
}

void VelmaGazebo::setInitialPosition(const std::map<std::string, double> &init_q) {
//    double angle = 90.0/180.0*3.1415;

    std::vector<gazebo::physics::JointPtr>  joints = model_->GetJoints();
    for (int i=0; i<joints.size(); i++) {
        std::map<std::string, double>::const_iterator it = init_q.find(joints[i]->GetName());
        if (it == init_q.end()) {
            joints[i]->SetPosition(0, 0);
        }
        else {
            joints[i]->SetPosition(0, it->second);
        }
        joints[i]->SetVelocity(0, 0);
    }

/*
    r_dart_joints_[1]->setPosition(0, -angle);
    l_dart_joints_[1]->setPosition(0, angle);

    r_dart_joints_[2]->setPosition(0, angle);
    l_dart_joints_[2]->setPosition(0, -angle);

    r_dart_joints_[3]->setPosition(0, angle);
    l_dart_joints_[3]->setPosition(0, -angle);

    r_dart_joints_[5]->setPosition(0, -angle);
    l_dart_joints_[5]->setPosition(0, angle);
*/
}

void VelmaGazebo::setJointsDisabledPID() {
    jc_->Reset();

    std::string joint_names[15] = {"torso_0_joint", "right_arm_0_joint", "right_arm_1_joint",
        "right_arm_2_joint", "right_arm_3_joint", "right_arm_4_joint", "right_arm_5_joint",
        "right_arm_6_joint", "left_arm_0_joint", "left_arm_1_joint", "left_arm_2_joint",
        "left_arm_3_joint", "left_arm_4_joint", "left_arm_5_joint", "left_arm_6_joint"};

    for (int i=0; i<15; i++) {
        gazebo::physics::JointPtr joint = model_->GetJoint(joint_names[i]);
        jc_->AddJoint(joint);
        jc_->SetPositionPID(joint->GetScopedName(), gazebo::common::PID(10.0, 0, 0.0, 1.1, -1.1, 10.0,-10.0));
        jc_->SetPositionTarget(joint->GetScopedName(), joint->GetAngle(0).Radian());
    }
/*
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
*/

    jc_->AddJoint(head_pan_joint_);
    jc_->SetPositionPID(head_pan_joint_->GetScopedName(), gazebo::common::PID(1.0, 0.5, 0.0, 0.1, -0.1, 1.0,-1.0));
    jc_->SetPositionTarget(head_pan_joint_->GetScopedName(), head_pan_joint_->GetAngle(0).Radian());

    jc_->AddJoint(head_tilt_joint_);
    jc_->SetPositionPID(head_tilt_joint_->GetScopedName(), gazebo::common::PID(1.0, 0.5, 0.0, 0.1, -0.1, 1.0,-1.0));
    jc_->SetPositionTarget(head_tilt_joint_->GetScopedName(), head_tilt_joint_->GetAngle(0).Radian());

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
    getMassJointPositions(mass_q_);
    dyn_model_.computeM(mass_q_);


    // mass matrix
    const Eigen::MatrixXd &mass_matrix = dyn_model_.getM();
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            tmp_r_MassMatrix_out_(i,j) = mass_matrix(i+1, j+1);
            tmp_l_MassMatrix_out_(i,j) = mass_matrix(i+8, j+8);
        }
    }

    // gravity forces
    Eigen::VectorXd grav(15);
    grav.setZero();
    getGravComp(grav);


/*    if (!model_dart_) {
        return;
    }

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
*/

    Eigen::VectorXd ext_f(14);
    getExternalForces(ext_f);
//TODO: dart:    const Eigen::VectorXd &ext_f = dart_sk_->getExternalForces() + dart_sk_->getConstraintForces() + dart_sk_->getCoriolisForces();

    // external forces
    for (int i = 0; i < 7; i++) {
        tmp_r_JointTorque_out_(i) = 0;//ext_f[i];
        tmp_l_JointTorque_out_(i) = 0;//ext_f[i+7];
    }

    Eigen::VectorXd q(15), dq(15);
    getJointPositionAndVelocity(q, dq);

    const double torso_gear = 158.0;
    const double torso_trans_mult = 20000.0 * torso_gear / (M_PI * 2.0);
    const double torso_motor_offset = -88524.0;
    const double torso_joint_offset = 0;
    const double torso_motor_constant = 0.00105;

    // joint position
    for (int i = 0; i < 7; i++) {
        tmp_r_JointPosition_out_(i) = q(i+1);
        tmp_l_JointPosition_out_(i) = q(i+8);
    }
    tmp_t_MotorPosition_out_ = (q(0) - torso_joint_offset) * torso_trans_mult + torso_motor_offset;

    // joint velocity
    for (int i = 0; i < 7; i++) {
        tmp_r_JointVelocity_out_(i) = dq(i+1);
        tmp_l_JointVelocity_out_(i) = dq(i+8);
    }
    tmp_t_MotorVelocity_out_ = dq(0) * torso_trans_mult;

/*
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

    bool tmp_r_command_mode;
    bool tmp_l_command_mode;

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

        tmp_r_command_mode = r_command_mode_;
        tmp_l_command_mode = l_command_mode_;

        r_JointTorqueCommand_in_.setZero();
        l_JointTorqueCommand_in_.setZero();
        t_MotorCurrentCommand_in_ = 0.0;
    }

    // torque command
    if (tmp_r_command_mode) {
        for (int i = 0; i < 7; i++) {
            grav(i+1) += tmp_r_JointTorqueCommand_in_(i);
        }
    }

    if (tmp_l_command_mode) {
        for (int i = 0; i < 7; i++) {
            grav(i+8) += tmp_l_JointTorqueCommand_in_(i);
        }
    }

    grav(0) += tmp_t_MotorCurrentCommand_in_ * torso_gear * torso_motor_constant;

    setForces(grav);
//    dart_sk_->setForces(grav);

    jc_->SetPositionTarget(head_pan_joint_->GetScopedName(), -tmp_hp_q_in_ / head_trans);
    jc_->SetPositionTarget(head_tilt_joint_->GetScopedName(), tmp_ht_q_in_ / head_trans);

    jc_->Update();
}

