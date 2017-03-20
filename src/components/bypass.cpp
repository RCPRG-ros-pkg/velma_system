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

#include <vector>
#include <string>

#include "rtt/RTT.hpp"
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

#include "velma_core_cs_ve_body_msgs/Command.h"
#include "velma_core_cs_ve_body_msgs/StatusSC.h"
#include "velma_core_ve_body_re_body_msgs/Command.h"

using namespace RTT;

namespace velma_core_ve_body_types {

class BypassComponent: public RTT::TaskContext {
public:
    explicit BypassComponent(const std::string &name);

    bool configureHook();

    bool startHook();

    void stopHook();

    void updateHook();

    std::string getDiag();

private:


//    RTT::InputPort<uint32_t > port_cmd_test_in_;
    RTT::InputPort<double > port_cmd_tMotor_i_in_;
    RTT::InputPort<velma_core_ve_body_re_body_msgs::CommandArm > port_cmd_lArm_in_;
    RTT::InputPort<velma_core_ve_body_re_body_msgs::CommandArm > port_cmd_rArm_in_;
//    RTT::InputPort<int32_t > port_cmd_sc_in_;
    RTT::InputPort<int32_t > port_cmd_tact_in_;
    RTT::InputPort<double > port_cmd_hpMotor_q_in_;
    RTT::InputPort<double > port_cmd_htMotor_q_in_;
    RTT::InputPort<velma_core_ve_body_re_body_msgs::CommandHand > port_cmd_lHand_in_;
    RTT::InputPort<velma_core_ve_body_re_body_msgs::CommandHand > port_cmd_rHand_in_;

//    velma_core_cs_ve_body_msgs::Command cmd_in_;
//    RTT::InputPort<velma_core_cs_ve_body_msgs::Command > port_cmd_in_;

    velma_core_ve_body_re_body_msgs::Command cmd_out_;
    RTT::OutputPort<velma_core_ve_body_re_body_msgs::Command > port_cmd_out_;

    velma_core_cs_ve_body_msgs::StatusSC sc_out_;
    RTT::OutputPort<velma_core_cs_ve_body_msgs::StatusSC> port_sc_out_;


    int diag_;
};

BypassComponent::BypassComponent(const std::string &name)
    : TaskContext(name, PreOperational)
//    , port_cmd_in_("command_INPORT")
//    , port_cmd_test_in_("cmd_test_INPORT")
    , port_cmd_tMotor_i_in_("cmd_tMotor_INPORT")
    , port_cmd_lArm_in_("cmd_lArm_INPORT")
    , port_cmd_rArm_in_("cmd_rArm_INPORT")
//    , port_cmd_sc_in_("cmd_sc_INPORT")
    , port_cmd_tact_in_("cmd_tact_INPORT")
    , port_cmd_hpMotor_q_in_("cmd_hpMotor_q_INPORT")
    , port_cmd_htMotor_q_in_("cmd_htMotor_q_INPORT")
    , port_cmd_lHand_in_("cmd_lHand_INPORT")
    , port_cmd_rHand_in_("cmd_rHand_INPORT")
    , port_cmd_out_("command_OUTPORT")
{

//    this->ports()->addPort(port_cmd_test_in_);
    this->ports()->addPort(port_cmd_tMotor_i_in_);
    this->ports()->addPort(port_cmd_lArm_in_);
    this->ports()->addPort(port_cmd_rArm_in_);
//    this->ports()->addPort(port_cmd_sc_in_);
    this->ports()->addPort(port_cmd_tact_in_);
    this->ports()->addPort(port_cmd_hpMotor_q_in_);
    this->ports()->addPort(port_cmd_htMotor_q_in_);
    this->ports()->addPort(port_cmd_lHand_in_);
    this->ports()->addPort(port_cmd_rHand_in_);

//    this->ports()->addPort(port_cmd_in_);
    this->ports()->addPort(port_cmd_out_);
    this->ports()->addPort("sc_OUTPORT", port_sc_out_);

    this->addOperation("getDiag", &BypassComponent::getDiag, this, RTT::ClientThread);
}

std::string BypassComponent::getDiag() {
// this method may not be RT-safe
    int diag = diag_;

    if (diag == 1) {
        return "could not read commands";
    }
    return "";
}

bool BypassComponent::configureHook() {
    return true;
}

bool BypassComponent::startHook() {
    return true;
}

void BypassComponent::stopHook() {
}

void BypassComponent::updateHook() {

//    uint32_t test_in;
    double tMotor_i_in;
    velma_core_ve_body_re_body_msgs::CommandArm lArm_in;
    velma_core_ve_body_re_body_msgs::CommandArm rArm_in;
//    int32_t sc_in;
    int32_t tact_in;
    double hpMotor_q_in_;
    double htMotor_q_in_;
    velma_core_ve_body_re_body_msgs::CommandHand lHand_in;
    velma_core_ve_body_re_body_msgs::CommandHand rHand_in;

//    if (port_cmd_test_in_.read(test_in) != RTT::NewData) {
//        Logger::In in("BypassComponent::updateHook");
//        Logger::log() << Logger::Error << "could not read data on port " << port_cmd_test_in_.getName() << Logger::endl;
//        error();
//        return;
//    }

    if (port_cmd_tMotor_i_in_.read(tMotor_i_in) != RTT::NewData) {
        Logger::In in("BypassComponent::updateHook");
        Logger::log() << Logger::Error << "could not read data on port " << port_cmd_tMotor_i_in_.getName() << Logger::endl;
        error();
        return;
    }

    if (port_cmd_lArm_in_.read(lArm_in) != RTT::NewData) {
        Logger::In in("BypassComponent::updateHook");
        Logger::log() << Logger::Error << "could not read data on port " << port_cmd_lArm_in_.getName() << Logger::endl;
        error();
        return;
    }

    if (port_cmd_rArm_in_.read(rArm_in) != RTT::NewData) {
        Logger::In in("BypassComponent::updateHook");
        Logger::log() << Logger::Error << "could not read data on port " << port_cmd_rArm_in_.getName() << Logger::endl;
        error();
        return;
    }

    if (port_cmd_hpMotor_q_in_.read(hpMotor_q_in_) != RTT::NewData) {
        Logger::In in("BypassComponent::updateHook");
        Logger::log() << Logger::Error << "could not read data on port " << port_cmd_hpMotor_q_in_.getName() << Logger::endl;
        error();
        return;
    }

    if (port_cmd_htMotor_q_in_.read(htMotor_q_in_) != RTT::NewData) {
        Logger::In in("BypassComponent::updateHook");
        Logger::log() << Logger::Error << "could not read data on port " << port_cmd_htMotor_q_in_.getName() << Logger::endl;
        error();
        return;
    }


//    if (port_cmd_sc_in_ != RTT::NewData) {
//        Logger::In in("BypassComponent::updateHook");
//        Logger::log() << Logger::Error << "could not read data on port " << port_cmd_sc_in_.getName() << Logger::endl;
//        error();
//        return;
//    }

    cmd_out_ = velma_core_ve_body_re_body_msgs::Command();

    if (port_cmd_tact_in_.read(tact_in) != RTT::NewData) {
        cmd_out_.tact_valid = false;
    }
    else {
        cmd_out_.tact = tact_in;
        cmd_out_.tact_valid = true;
    }

    if (port_cmd_lHand_in_.read(lHand_in) != RTT::NewData) {
        cmd_out_.lHand_valid = false;
    }
    else {
        cmd_out_.lHand = lHand_in;
        cmd_out_.lHand_valid = true;
    }

    if (port_cmd_rHand_in_.read(rHand_in) != RTT::NewData) {
        cmd_out_.rHand_valid = false;
    }
    else {
        cmd_out_.rHand = rHand_in;
        cmd_out_.rHand_valid = true;
    }

//    if (port_cmd_in_.read(cmd_in_) != RTT::NewData) {
//        Logger::In in("BypassComponent::updateHook");
//        Logger::log() << Logger::Error << "could not read data on port "
//            << port_cmd_in_.getName() << Logger::endl;
//        error();
//        diag_ = 1;
//        return;
//    }

    diag_ = 0;

    cmd_out_.tMotor_i = tMotor_i_in;
    cmd_out_.tMotor_i_valid = true;

    cmd_out_.hpMotor_valid = true;
    cmd_out_.hpMotor.q_valid = true;
    cmd_out_.hpMotor.q = hpMotor_q_in_;

    cmd_out_.htMotor_valid = true;
    cmd_out_.htMotor.q_valid = true;
    cmd_out_.htMotor.q = htMotor_q_in_;

    cmd_out_.lArm = lArm_in;
    cmd_out_.lArm_valid = true;

    cmd_out_.rArm = rArm_in;
    cmd_out_.rArm_valid = true;

    port_cmd_out_.write(cmd_out_);

    // no error
    sc_out_.safe_behavior = false;
    sc_out_.error = false;
    sc_out_.fault_type = 0;
    sc_out_.faulty_module_id = 0;

    port_sc_out_.write(sc_out_);

// TODO:
//CommandMotor hpMotor    # subsystem_buffer{type: container; validity: hpMotor_valid}
//bool hpMotor_valid
//CommandMotor htMotor    # subsystem_buffer{type: container; validity: htMotor_valid}
//bool htMotor_valid

}

}   //namespace velma_core_ve_body_types

ORO_LIST_COMPONENT_TYPE(velma_core_ve_body_types::BypassComponent)

