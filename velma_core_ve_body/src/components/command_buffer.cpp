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

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

#include <velma_core_cs_ve_body_msgs/Command.h>

#include "../common_predicates.h"

using namespace RTT;

namespace velma_core_ve_body_types {

class CommandBuffer: public RTT::TaskContext {
public:
    explicit CommandBuffer(const std::string &name);

    bool startHook();

    void stopHook();

    void updateHook();

private:
    // ports
    velma_core_cs_ve_body_msgs::Command cmd_in_;
    RTT::InputPort<velma_core_cs_ve_body_msgs::Command > port_cmd_in_;

    bool rLwrCmdOk_;
    bool lLwrCmdOk_;
    bool tCmdOk_;
};

CommandBuffer::CommandBuffer(const std::string &name)
    : TaskContext(name)
    , rLwrCmdOk_(false)
    , lLwrCmdOk_(false)
    , tCmdOk_(false)
{
    this->ports()->addPort("cmd_INPORT", port_cmd_in_);

    this->addAttribute("rLwrCmdOk", rLwrCmdOk_);
    this->addAttribute("lLwrCmdOk", lLwrCmdOk_);
    this->addAttribute("tCmdOk", tCmdOk_);
}

bool CommandBuffer::startHook() {
    return true;
}

void CommandBuffer::stopHook() {
}

void CommandBuffer::updateHook() {
    bool read_cmd = (port_cmd_in_.read(cmd_in_) == RTT::NewData);
    rLwrCmdOk_ = true;//read_cmd && cmd_in_.rArm_valid && isCmdArmValid(cmd_in_.rArm);
    lLwrCmdOk_ = true;//read_cmd && cmd_in_.lArm_valid && isCmdArmValid(cmd_in_.lArm);
    tCmdOk_ = true;//read_cmd && cmd_in_.tMotor_i_valid && isCmdTorsoValid(cmd_in_.tMotor_i);
}

}   // velma_core_ve_body_types

ORO_LIST_COMPONENT_TYPE(velma_core_ve_body_types::CommandBuffer)

