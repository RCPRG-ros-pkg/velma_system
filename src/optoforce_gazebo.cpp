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

#include "optoforce_gazebo.h"
#include "rtt_rosclock/rtt_rosclock.h"

    OptoforceGazebo::OptoforceGazebo(std::string const& name) : 
        TaskContext(name),
        model_(NULL),
        n_sensors_(0)
    {
        nh_ = new ros::NodeHandle();

        // Add required gazebo interfaces
        this->provides("gazebo")->addOperation("configure",&OptoforceGazebo::gazeboConfigureHook,this,RTT::ClientThread);
        this->provides("gazebo")->addOperation("update",&OptoforceGazebo::gazeboUpdateHook,this,RTT::ClientThread);

        this->addProperty("device_name", device_name_);
        this->addProperty("n_sensors", n_sensors_);
        this->addProperty("frame_id_vec", frame_id_vec_);
    }

    OptoforceGazebo::~OptoforceGazebo() {
    }

    bool OptoforceGazebo::configureHook() {
        if(model_.get() == NULL) {
            std::cout << "ERROR: OptoforceGazebo::configureHook: gazebo model is NULL" << std::endl;
            return false;
        }

        if (frame_id_vec_.size() != n_sensors_) {
            std::cout << "ERROR: OptoforceGazebo::configureHook: frame_id_vec_.size() != n_sensors_   " << frame_id_vec_.size() << "!=" << n_sensors_ << std::endl;
            return false;
        }

        if (n_sensors_ == 1) {
            std::cout << "ERROR: OptoforceGazebo::configureHook: not implemented for n_sensors==1" << std::endl;
            return false;
        }
        else if (n_sensors_ == 3) {
            std::string prefix;
            if (device_name_ == "gazebo_rightHand") {
                prefix = "right";
            }
            else if (device_name_ == "gazebo_leftHand") {
                prefix = "left";
            }
            else {
                std::cout << "ERROR: OptoforceGazebo::configureHook: wrong device_name=\"" << device_name_ << "\". Should be \"gazebo_rightHand\" or \"gazebo_leftHand\"" << std::endl;
                return false;
            }

            const int n_joints = 3;
            std::string joint_names[n_joints] = { prefix + std::string("_HandFingerOneKnuckleThreeOptoforceJoint"),
                prefix + std::string("_HandFingerTwoKnuckleThreeOptoforceJoint"),
                prefix + std::string("_HandFingerThreeKnuckleThreeOptoforceJoint") };

            for (int i=0; i < n_joints; i++) {
                gazebo::physics::JointPtr jnt = model_->GetJoint(joint_names[i]);
                if (jnt.get() == NULL) {
                    std::cout << "ERROR: OptoforceGazebo::configureHook: could not find the joint " << joint_names[i] << std::endl;
                    return false;
                }
                joints_.push_back( jnt );
                jc_->AddJoint(jnt);
                jc_->SetPositionPID(jnt->GetScopedName(), gazebo::common::PID(20.0, 0, 0.0, 0, 0, 20.0,-20.0));
                jc_->SetPositionTarget(jnt->GetScopedName(), 0.0);
            }
        }
        else {
            std::cout << "ERROR: OptoforceGazebo::configureHook: wrong n_sensors=" << n_sensors_ << ". Should be 1 or 3." << std::endl;
            return false;
        }

        port_force_out_.resize(n_sensors_);

        for (size_t i = 0; i < n_sensors_; i++) {
            char name[30];
            snprintf(name, sizeof(name), "force_%zu_OUTPORT", i);
            port_force_out_[i] = new RTT::OutputPort<geometry_msgs::WrenchStamped >();
            this->ports()->addPort(name, *port_force_out_[i]);
        }

        force_out_.resize(n_sensors_);

        std::cout << "OptoforceGazebo::configureHook: ok" << std::endl;
        return true;
    }

    void OptoforceGazebo::updateHook() {
        // Synchronize with gazeboUpdate()
        RTT::os::MutexLock lock(gazebo_mutex_);

        for (int i = 0; i < n_sensors_; i++) {
            port_force_out_[i]->write(force_out_[i]);
        }
    }

    bool OptoforceGazebo::startHook() {
      return true;
    }

    bool OptoforceGazebo::gazeboConfigureHook(gazebo::physics::ModelPtr model) {

        if(model.get() == NULL) {
            std::cout << "OptoforceGazebo::gazeboConfigureHook: the gazebo model is NULL" << std::endl;
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

        detector_ = dart_world_->getConstraintSolver()->getCollisionDetector();

        jc_.reset(new gazebo::physics::JointController(model_));

        return true;
    }

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void OptoforceGazebo::gazeboUpdateHook(gazebo::physics::ModelPtr model)
{
    RTT::os::MutexTryLock trylock(gazebo_mutex_);
    if(!trylock.isSuccessful()) {
        return;
    }

    if (n_sensors_ == 0 || joints_.size() != n_sensors_) {
//        std::cout << "ERROR: OptoforceGazebo: joints_.size() != " << n_sensors_ << std::endl;
        return;
    }

    for (int i = 0; i < n_sensors_; i++) {
        force_out_[i].header.frame_id = frame_id_vec_[i];
        force_out_[i].header.stamp = rtt_rosclock::host_now();
        gazebo::physics::JointWrench wr = joints_[i]->GetForceTorque(0u);
        force_out_[i].wrench.force.x = wr.body2Force.x;
        force_out_[i].wrench.force.y = wr.body2Force.y;
        force_out_[i].wrench.force.z = wr.body2Force.z;
        force_out_[i].wrench.torque.x = force_out_[i].wrench.torque.y = force_out_[i].wrench.torque.z = 0.0;
    }

    jc_->Update();
}

ORO_LIST_COMPONENT_TYPE(OptoforceGazebo)
ORO_CREATE_COMPONENT_LIBRARY();

