#include "barrett_hand_gazebo.h"

    void BarrettHandGazebo::updateHook() {
        // Synchronize with gazeboUpdate()
        RTT::os::MutexLock lock(gazebo_mutex_);

        //
        // BarrettHand
        //
        port_q_out_.write(q_out_);
        port_t_out_.write(t_out_);

        port_status_out_.write(status_out_);

        if (port_q_in_.read(q_in_) == RTT::NewData) {
//            std::cout << "q_in_: new data " << q_in_.transpose() << std::endl;
            status_out_ = 0;
            move_hand_ = true;
        }
        port_v_in_.read(v_in_);
        port_t_in_.read(t_in_);
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
            std::string name( prefix_ + hand_joint_names[i] );
            dart_sk_->getJoint(name)->setActuatorType( dart::dynamics::Joint::FORCE );
            gazebo::physics::JointPtr joint = model_->GetJoint(name);
            joints_.push_back(joint);
            dart::dynamics::Joint* joint_dart = dart_sk_->getJoint(name);
            joints_dart_.push_back( joint_dart );
            joint->SetEffortLimit(0, 1);
        }

        jc_ = new gazebo::physics::JointController(model_);

        for (int i = 0; i < 3; i++) {
            clutch_break_[i] = false;
        }

        for (int i = 0; i < 8; i++) {
            jc_->AddJoint(joints_[i]);
        }
        jc_->SetVelocityPID(joints_[0]->GetScopedName(), gazebo::common::PID(1.0, 0.0, 0.0, 0.0, 0.0, 1.0,-1.0));
        jc_->SetVelocityPID(joints_[3]->GetScopedName(), gazebo::common::PID(1.0, 0.0, 0.0, 0.0, 0.0, 1.0,-1.0));
        jc_->SetVelocityPID(joints_[1]->GetScopedName(), gazebo::common::PID(0.5, 0.0, 0.0, 0.0, 0.0, 0.5,-0.5));
        jc_->SetVelocityPID(joints_[4]->GetScopedName(), gazebo::common::PID(0.5, 0.0, 0.0, 0.0, 0.0, 0.5,-0.5));
        jc_->SetVelocityPID(joints_[6]->GetScopedName(), gazebo::common::PID(0.5, 0.0, 0.0, 0.0, 0.0, 0.5,-0.5));
        jc_->SetVelocityPID(joints_[2]->GetScopedName(), gazebo::common::PID(0.2, 0.0, 0.0, 0.0, 0.0, 0.2,-0.2));
        jc_->SetVelocityPID(joints_[5]->GetScopedName(), gazebo::common::PID(0.2, 0.0, 0.0, 0.0, 0.0, 0.2,-0.2));
        jc_->SetVelocityPID(joints_[7]->GetScopedName(), gazebo::common::PID(0.2, 0.0, 0.0, 0.0, 0.0, 0.2,-0.2));

        std::cout << "BarrettHandGazebo::configureHook(" << prefix_ << "): ok " << std::endl;
        return true;
    }

