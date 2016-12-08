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

#include "eigen_conversions/eigen_msg.h"

#include <math.h>

#include <rtt_rosclock/rtt_rosclock.h>

#include <sys/time.h>

using namespace RTT;

namespace velma_core_cs_types {

class SafeComponent: public RTT::TaskContext {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit SafeComponent(const std::string &name);

    bool configureHook();

    bool startHook();

    void stopHook();

    void updateHook();

    std::string getDiag();

private:

    // OROCOS ports
    velma_core_cs_ve_body_msgs::Command cmd_out_;
    RTT::OutputPort<velma_core_cs_ve_body_msgs::Command > port_cmd_out_;

    velma_core_cs_ve_body_msgs::Status status_in_;
    RTT::InputPort<velma_core_cs_ve_body_msgs::Status > port_status_in_;
};

SafeComponent::SafeComponent(const std::string &name) :
    TaskContext(name, PreOperational)
{
    this->ports()->addPort("status_INPORT", port_status_in_);
    this->ports()->addPort("cmd_OUTPORT", port_cmd_out_);

    // TODO
    //this->addOperation("getDiag", &SafeComponent::getDiag, this, RTT::ClientThread);
}

std::string SafeComponent::getDiag() {
// this method may not be RT-safe
    return "TODO";
}

bool SafeComponent::configureHook() {
    Logger::In in("SafeComponent::configureHook");

    return true;
}

bool SafeComponent::startHook() {
    return true;
}

void SafeComponent::stopHook() {
}

void SafeComponent::updateHook() {
    //
    // read HW status
    //
    if (port_status_in_.read(status_in_) != RTT::NewData) {
        status_in_ = velma_core_cs_ve_body_msgs::Status();
    }

    // set all commands to zero
    cmd_out_ = velma_core_cs_ve_body_msgs::Command();

    cmd_out_.test = status_in_.test;

    // zero torque for hands
    cmd_out_.rArm_valid = true;
    cmd_out_.lArm_valid = true;

    // zero torque for torso
    cmd_out_.tMotor_valid = true;

    // keep current position of neck
    cmd_out_.hpMotor.q = status_in_.hpMotor.q;
    cmd_out_.hpMotor_valid = status_in_.hpMotor_valid;

    cmd_out_.htMotor.q = status_in_.htMotor.q;
    cmd_out_.htMotor_valid = status_in_.htMotor_valid;

    if (status_in_.sc_valid && status_in_.sc.safe_behavior && !status_in_.sc.error) {
        cmd_out_.sc.cmd = 1;
        cmd_out_.sc.valid = true;   // TODO: this is not needed
        cmd_out_.sc_valid = true;
    }

    //
    // write commands
    //
    port_cmd_out_.write(cmd_out_);
}

}   // namespace velma_core_cs_types

ORO_LIST_COMPONENT_TYPE(velma_core_cs_types::SafeComponent)

