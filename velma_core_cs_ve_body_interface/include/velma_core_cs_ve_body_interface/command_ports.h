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

#ifndef __VELMA_CORE_CS_VE_BODY_COMMAND_PORTS_H__
#define __VELMA_CORE_CS_VE_BODY_COMMAND_PORTS_H__

#include "rtt/RTT.hpp"

#include "velma_core_cs_ve_body_msgs/Command.h"
#include "velma_core_ve_body_re_body_msgs/CommandArm.h"
#include "velma_core_ve_body_re_body_msgs/CommandHand.h"
#include "velma_core_ve_body_re_body_msgs/CommandMotor.h"
#include "velma_core_ve_body_re_body_msgs/CommandSimple.h"

#include "common_interfaces/interface_ports.h"

#include "velma_core_cs_ve_body_interface/port_data.h"

namespace velma_core_cs_ve_body_interface {

using namespace interface_ports;
using velma_core_cs_ve_body_msgs::Command;
using velma_core_ve_body_re_body_msgs::CommandArm;
using velma_core_ve_body_re_body_msgs::CommandHand;
using velma_core_ve_body_re_body_msgs::CommandSimple;
using velma_core_ve_body_re_body_msgs::CommandMotor;

template <template <typename Type> class T>
class ArmCommand_Ports : public PortsContainer<Command, CommandArm > {
public:
    ArmCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, CommandArm Command::*ptr);
};

template <template <typename Type> class T>
class HandCommand_Ports : public PortsContainer<Command, CommandHand > {
public:
    HandCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, CommandHand Command::*ptr);
};

template <template <typename Type> class T>
class SimpleCommand_Ports : public PortsContainer<Command, CommandSimple > {
public:
    SimpleCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, CommandSimple Command::*ptr);
};

template <template <typename Type> class T>
class MotorCommand_Ports : public PortsContainer<Command, CommandMotor > {
public:
    MotorCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, CommandMotor Command::*ptr);
};


template <template <typename Type> class T>
class FTSensorCommand_Ports {
public:
    FTSensorCommand_Ports(RTT::TaskContext &tc, const std::string &prefix);
};

template <template <typename Type> class T>
class VelmaCommand_Ports : public PortsContainer<void, Command > {
public:
    typedef Command Container;
    VelmaCommand_Ports(RTT::TaskContext &tc);
};

template class VelmaCommand_Ports<RTT::InputPort >;
template class VelmaCommand_Ports<RTT::OutputPort >;

};  // namespace velma_core_cs_ve_body_interface


#endif  // __VELMA_CORE_CS_VE_BODY_COMMAND_PORTS_H__

