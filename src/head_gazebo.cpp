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

#include "rtt_rosclock/rtt_rosclock.h"
#include <rtt/Logger.hpp>
#include "velma_sim_conversion.h"

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "Eigen/Dense"

#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>

#include <geometry_msgs/Wrench.h>

#include <kuka_lwr_fri/friComm.h>

class HeadGazebo : public RTT::TaskContext
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<double, 2, 1> Joints;
    
    // public methods
    HeadGazebo(std::string const& name);
    ~HeadGazebo();
    void updateHook();
    bool startHook();
    bool configureHook();
    bool gazeboConfigureHook(gazebo::physics::ModelPtr model);
    void gazeboUpdateHook(gazebo::physics::ModelPtr model);

  protected:

    // head ports
    RTT::InputPort<Joints >      port_q_in_;
    RTT::OutputPort<Joints >     port_q_out_;
    RTT::OutputPort<Joints >     port_dq_out_;

    Joints q_in_;
    Joints q_out_;
    Joints dq_out_;

    Joints tmp_q_in_;
    Joints tmp_q_out_;
    Joints tmp_dq_out_;

    void setJointsPID();

    ros::NodeHandle *nh_;

    gazebo::physics::ModelPtr model_;

    // head
    gazebo::physics::JointPtr head_pan_joint_;
    gazebo::physics::JointPtr head_tilt_joint_;

    std::string head_pan_scoped_name_;
    std::string head_tilt_scoped_name_;

    gazebo::physics::JointController *jc_;

    void getHeadJointPositionAndVelocity(Joints &q, Joints &dq);

    //! Synchronization
    RTT::os::MutexRecursive gazebo_mutex_;

    bool data_valid_;
};

using namespace RTT;

HeadGazebo::HeadGazebo(std::string const& name)
    : TaskContext(name, RTT::TaskContext::PreOperational)
    , data_valid_(false)
    , q_in_(2)
    , q_out_(2)
    , dq_out_(2)
    , port_q_in_("q_INPORT")
    , port_q_out_("q_OUTPORT")
    , port_dq_out_("dq_OUTPORT")
{

    nh_ = new ros::NodeHandle();

    // Add required gazebo interfaces
    this->provides("gazebo")->addOperation("configure",&HeadGazebo::gazeboConfigureHook,this,RTT::ClientThread);
    this->provides("gazebo")->addOperation("update",&HeadGazebo::gazeboUpdateHook,this,RTT::ClientThread);

    this->ports()->addPort(port_q_in_);
    this->ports()->addPort(port_q_out_);
    this->ports()->addPort(port_dq_out_);

    q_in_.setZero();
    q_out_.setZero();
    dq_out_.setZero();
}

HeadGazebo::~HeadGazebo() {
}

void HeadGazebo::getHeadJointPositionAndVelocity(HeadGazebo::Joints &q, HeadGazebo::Joints &dq) {
    q(0) = head_pan_joint_->GetAngle(0).Radian();
    dq(0) = head_pan_joint_->GetVelocity(0);

    q(1) = head_tilt_joint_->GetAngle(0).Radian();
    dq(1) = head_tilt_joint_->GetVelocity(0);
}

bool HeadGazebo::gazeboConfigureHook(gazebo::physics::ModelPtr model) {
    Logger::In in("HeadGazebo::gazeboConfigureHook");

    if(model.get() == NULL) {
        Logger::log() << Logger::Error << "gazebo model is NULL" << Logger::endl;
        return false;
    }

    model_ = model;

    jc_ = new gazebo::physics::JointController(model);

    // head joints
    head_pan_joint_ = model->GetJoint("head_pan_joint");
    head_tilt_joint_ = model->GetJoint("head_tilt_joint");

    head_pan_scoped_name_ = head_pan_joint_->GetScopedName();
    head_tilt_scoped_name_ = head_tilt_joint_->GetScopedName();

    return true;
}

void HeadGazebo::setJointsPID() {
    jc_->Reset();

    jc_->AddJoint(head_pan_joint_);
    jc_->SetPositionPID(head_pan_scoped_name_, gazebo::common::PID(1.0, 0.5, 0.0, 0.1, -0.1, 1.0,-1.0));
    jc_->SetPositionTarget(head_pan_scoped_name_, head_pan_joint_->GetAngle(0).Radian());

    jc_->AddJoint(head_tilt_joint_);
    jc_->SetPositionPID(head_tilt_scoped_name_, gazebo::common::PID(1.0, 0.5, 0.0, 0.1, -0.1, 1.0,-1.0));
    jc_->SetPositionTarget(head_tilt_scoped_name_, head_tilt_joint_->GetAngle(0).Radian());

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void HeadGazebo::gazeboUpdateHook(gazebo::physics::ModelPtr model)
{
    Logger::In in("HeadGazebo::gazeboUpdateHook");

    Joints q, dq;

    getHeadJointPositionAndVelocity(q, dq);

    const double head_trans = 8000.0 * 100.0 / (M_PI * 2.0);
    tmp_q_out_(0) = -q(0) * head_trans;
    tmp_q_out_(1) = q(1) * head_trans;
    tmp_dq_out_(0) = dq(0); // TODO: * head_trans ???
    tmp_dq_out_(1) = dq(1); // TODO: * head_trans ???

    // exchange the data between Orocos and Gazebo
    {
        RTT::os::MutexLock lock(gazebo_mutex_);

        q_out_ = tmp_q_out_;
        dq_out_ = tmp_dq_out_;

        tmp_q_in_ = q_in_;

        data_valid_ = true;
    }

    // joint controller for the head
    jc_->SetPositionTarget(head_pan_scoped_name_, -tmp_q_in_(0) / head_trans);
    jc_->SetPositionTarget(head_tilt_scoped_name_, tmp_q_in_(1) / head_trans);

    jc_->Update();
}

void HeadGazebo::updateHook() {

    // Synchronize with gazeboUpdate()
    RTT::os::MutexLock lock(gazebo_mutex_);

    if (!data_valid_) {
        return;
    }

    port_q_in_.read(q_in_);
    port_dq_out_.write(dq_out_);
    port_q_out_.write(q_out_);
}

bool HeadGazebo::startHook() {
    return true;
}

bool HeadGazebo::configureHook() {
    setJointsPID();
    return true;
}

ORO_LIST_COMPONENT_TYPE(HeadGazebo)

