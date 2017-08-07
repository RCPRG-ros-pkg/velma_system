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

#include <sstream>

#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>
#include <rtt/base/PortInterface.hpp>

#include "velma_core_cs_ve_body_msgs/Command.h"
#include "velma_core_cs_ve_body_msgs/Status.h"
#include "velma_core_cs_task_cs_msgs/Status.h"

#include "eigen_conversions/eigen_msg.h"

#include <math.h>

#include <rtt_rosclock/rtt_rosclock.h>

#include <sys/time.h>

using namespace RTT;

namespace velma_core_cs_types {

const int NUMBER_OF_JOINTS = 15;

class SafeComponent: public RTT::TaskContext {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit SafeComponent(const std::string &name);

    bool configureHook();

    bool startHook();

    void updateHook();

private:

    typedef Eigen::Matrix<double, NUMBER_OF_JOINTS, 1>  VectorNd;

    // OROCOS ports
    RTT::OutputPort<uint32_t > port_status_subsystem_state_out_;

    velma_core_cs_ve_body_msgs::Status status_in_;
    RTT::InputPort<velma_core_cs_ve_body_msgs::Status > port_status_in_;

    RTT::OutputPort<VectorNd> port_internal_space_position_command_out_;
    RTT::InputPort<VectorNd> port_internal_space_position_measurement_in_;

    VectorNd joint_stiffness_command_;
    RTT::OutputPort<VectorNd> port_joint_stiffness_command_;

    RTT::InputPort<double > port_ht_q_in_;
    RTT::InputPort<double > port_hp_q_in_;
    RTT::OutputPort<double > port_ht_q_out_;
    RTT::OutputPort<double > port_hp_q_out_;
    RTT::InputPort<uint8_t > port_ht_enabled_in_;
    RTT::InputPort<uint8_t > port_hp_enabled_in_;
    RTT::InputPort<uint8_t > port_ht_homing_in_progress_in_;
    RTT::InputPort<uint8_t > port_hp_homing_in_progress_in_;
    double hp_q_;
    double ht_q_;


    VectorNd internal_space_position_;

    bool first_step_;
};

SafeComponent::SafeComponent(const std::string &name)
    : TaskContext(name, PreOperational)
    , port_internal_space_position_command_out_("JointPositionCommand_OUTPORT")
    , port_internal_space_position_measurement_in_("JointPosition_INPORT")
    , port_joint_stiffness_command_("JointStiffnessCommand_OUTPORT")
    , port_status_in_("status_INPORT")
    , port_status_subsystem_state_out_("subsystem_state_OUTPORT")
    , first_step_(true)
{

    this->ports()->addPort(port_internal_space_position_command_out_);
    this->ports()->addPort(port_internal_space_position_measurement_in_);
    this->ports()->addPort(port_joint_stiffness_command_);
    this->ports()->addPort(port_status_in_);
    this->ports()->addPort(port_status_subsystem_state_out_);

    this->ports()->addPort("ht_q_INPORT", port_ht_q_in_);
    this->ports()->addPort("hp_q_INPORT", port_hp_q_in_);
    this->ports()->addPort("ht_q_OUTPORT", port_ht_q_out_);
    this->ports()->addPort("hp_q_OUTPORT", port_hp_q_out_);
    this->ports()->addPort("ht_enabled_INPORT", port_ht_enabled_in_);
    this->ports()->addPort("hp_enabled_INPORT", port_hp_enabled_in_);
    this->ports()->addPort("ht_homing_in_progress_INPORT", port_ht_homing_in_progress_in_);
    this->ports()->addPort("hp_homing_in_progress_INPORT", port_hp_homing_in_progress_in_);
}

bool SafeComponent::configureHook() {
    Logger::In in("SafeComponent::configureHook");

    for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
        joint_stiffness_command_(i) = 5.0;
    }

    return true;
}

bool SafeComponent::startHook() {
    first_step_ = true;
    return true;
}

void SafeComponent::updateHook() {
    //
    // read HW status
    //
    if (port_status_in_.read(status_in_) != RTT::NewData) {
        status_in_ = velma_core_cs_ve_body_msgs::Status();
    }

/*
    // set all commands to zero
    cmd_out_ = velma_core_cs_ve_body_msgs::Command();

    cmd_out_.test = status_in_.test;

    // zero torque for hands
    cmd_out_.rArm_valid = true;
    cmd_out_.lArm_valid = true;

    // zero torque for torso
    cmd_out_.tMotor_valid = true;
    cmd_out_.tMotor.i_valid = true;

    // keep current position of neck
    cmd_out_.hpMotor.q = status_in_.hpMotor.q;
    cmd_out_.hpMotor_valid = status_in_.hpMotor_valid;
    cmd_out_.hpMotor.q_valid = status_in_.hpMotor_valid;

    cmd_out_.htMotor.q = status_in_.htMotor.q;
    cmd_out_.htMotor_valid = status_in_.htMotor_valid;
    cmd_out_.htMotor.q_valid = status_in_.htMotor_valid;
*/

/*    if (status_in_.sc_valid && status_in_.sc.safe_behavior && !status_in_.sc.error) {
        cmd_sc_out_.cmd = 1;
        cmd_sc_out_.valid = true;   // TODO: this is not needed
        first_step_ = true;
    }
*/
    // read current configuration
    if (first_step_) {
        first_step_ = false;

        port_ht_q_in_.read(ht_q_);
        port_hp_q_in_.read(hp_q_);

        if (port_internal_space_position_measurement_in_.read(internal_space_position_) != RTT::NewData) {
            // the safety component cannot be started - there is a serious problem with subsystem structure
            error();
            return;
        }

        Eigen::Matrix<double, 7, 1> rArm, lArm;
        for (int i = 0; i < 7; ++i) {
            rArm(i) = status_in_.rArm.q[i];
            lArm(i) = status_in_.lArm.q[i];
        }
//        log(RTT::Error) << "first step " << internal_space_position_.transpose()
//            << " t: " << status_in_.tMotor.q
//            << " r: " << rArm.transpose()
//            << " l: " << lArm.transpose() << Logger::endl;
    }
    else {
        uint8_t ht_enabled;
        uint8_t ht_homing_in_progress;
        if (port_ht_enabled_in_.read(ht_enabled) == RTT::NewData && port_ht_homing_in_progress_in_.read(ht_homing_in_progress) == RTT::NewData) {
            if (!ht_enabled || ht_homing_in_progress) {
                port_ht_q_in_.read(ht_q_);
            }
        }
        uint8_t hp_enabled;
        uint8_t hp_homing_in_progress;
        if (port_hp_enabled_in_.read(hp_enabled) == RTT::NewData && port_hp_homing_in_progress_in_.read(hp_homing_in_progress) == RTT::NewData) {
            if (!hp_enabled || hp_homing_in_progress) {
                port_hp_q_in_.read(hp_q_);
            }
        }
    }

    port_ht_q_out_.write(ht_q_);
    port_hp_q_out_.write(hp_q_);

//    internal_space_position_.setZero();
    port_internal_space_position_command_out_.write(internal_space_position_);
    port_joint_stiffness_command_.write(joint_stiffness_command_);

    //
    // write status
    //
    port_status_subsystem_state_out_.write(velma_core_cs_task_cs_msgs::Status::STATE_SAFE);

}

}   // namespace velma_core_cs_types

ORO_LIST_COMPONENT_TYPE(velma_core_cs_types::SafeComponent)

