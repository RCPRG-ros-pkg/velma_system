#include "barrett_hand_gazebo.h"

    void BarrettHandGazebo::updateHook() {
        // Synchronize with gazeboUpdate()
        RTT::os::MutexLock lock(gazebo_mutex_);

        //
        // BarrettHand
        //
        port_rh_q_out_.write(rh_q_out_);
        port_rh_t_out_.write(rh_t_out_);

        port_rh_status_out_.write(rh_status_out_);

        if (port_rh_q_in_.read(rh_q_in_) == RTT::NewData) {
            std::cout << "rh_q_in_: new data " << rh_q_in_.transpose() << std::endl;
            rh_status_out_ = 0;
            rh_move_hand_ = true;
        }
        port_rh_v_in_.read(rh_v_in_);
        port_rh_t_in_.read(rh_t_in_);
    }

    bool BarrettHandGazebo::startHook() {
      return true;
    }

    bool BarrettHandGazebo::configureHook() {

        if (prefix_.empty()) {
            std::cout << "ERROR: BarrettHandGazebo::configureHook: prefix is empty" << std::endl;
            return false;
        }

        std::string hand_joint_names[] = {"_HandFingerOneKnuckleOneJoint", "_HandFingerOneKnuckleTwoJoint", "_HandFingerOneKnuckleThreeJoint",
            "_HandFingerTwoKnuckleOneJoint", "_HandFingerTwoKnuckleTwoJoint", "_HandFingerTwoKnuckleThreeJoint",
            "_HandFingerThreeKnuckleTwoJoint", "_HandFingerThreeKnuckleThreeJoint" };

        for (int i = 0; i < 8; i++) {
            std::string rh_name( prefix_ + hand_joint_names[i] );
            dart_sk_->getJoint(rh_name)->setActuatorType( dart::dynamics::Joint::FORCE );
            gazebo::physics::JointPtr rh_joint = model_->GetJoint(rh_name);
            rh_joints_.push_back(rh_joint);
            dart::dynamics::Joint* rh_joint_dart = dart_sk_->getJoint(rh_name);
            rh_joints_dart_.push_back( rh_joint_dart );
            rh_joint->SetEffortLimit(0, 1);
        }

        jc_ = new gazebo::physics::JointController(model_);

        for (int i = 0; i < 3; i++) {
            rh_clutch_break_[i] = false;
        }

        for (int i = 0; i < 8; i++) {
            jc_->AddJoint(rh_joints_[i]);
        }
        jc_->SetVelocityPID(rh_joints_[0]->GetScopedName(), gazebo::common::PID(1.0, 0.0, 0.0, 0.0, 0.0, 1.0,-1.0));
        jc_->SetVelocityPID(rh_joints_[3]->GetScopedName(), gazebo::common::PID(1.0, 0.0, 0.0, 0.0, 0.0, 1.0,-1.0));
        jc_->SetVelocityPID(rh_joints_[1]->GetScopedName(), gazebo::common::PID(0.5, 0.0, 0.0, 0.0, 0.0, 0.5,-0.5));
        jc_->SetVelocityPID(rh_joints_[4]->GetScopedName(), gazebo::common::PID(0.5, 0.0, 0.0, 0.0, 0.0, 0.5,-0.5));
        jc_->SetVelocityPID(rh_joints_[6]->GetScopedName(), gazebo::common::PID(0.5, 0.0, 0.0, 0.0, 0.0, 0.5,-0.5));
        jc_->SetVelocityPID(rh_joints_[2]->GetScopedName(), gazebo::common::PID(0.2, 0.0, 0.0, 0.0, 0.0, 0.2,-0.2));
        jc_->SetVelocityPID(rh_joints_[5]->GetScopedName(), gazebo::common::PID(0.2, 0.0, 0.0, 0.0, 0.0, 0.2,-0.2));
        jc_->SetVelocityPID(rh_joints_[7]->GetScopedName(), gazebo::common::PID(0.2, 0.0, 0.0, 0.0, 0.0, 0.2,-0.2));

        std::cout << "BarrettHandGazebo::configureHook(" << prefix_ << "): ok " << std::endl;
        return true;
    }

