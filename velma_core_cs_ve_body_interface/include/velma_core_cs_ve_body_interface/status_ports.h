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

#ifndef __VELMA_CORE_CS_VE_BODY_STATUS_PORTS_H__
#define __VELMA_CORE_CS_VE_BODY_STATUS_PORTS_H__

#include "rtt/RTT.hpp"

#include "velma_core_cs_ve_body_msgs/Status.h"
#include "velma_core_ve_body_re_body_msgs/StatusArm.h"
#include "velma_core_ve_body_re_body_msgs/StatusHand.h"
#include "velma_core_ve_body_re_body_msgs/StatusMotor.h"
#include "velma_core_ve_body_re_body_msgs/StatusFT.h"

#include "common_interfaces/interface_ports.h"

#include "velma_core_cs_ve_body_interface/port_data.h"

namespace velma_core_cs_ve_body_interface {

using namespace interface_ports;
using velma_core_cs_ve_body_msgs::Status;
using velma_core_ve_body_re_body_msgs::StatusArm;
using velma_core_ve_body_re_body_msgs::StatusHand;
using velma_core_ve_body_re_body_msgs::StatusMotor;
using velma_core_ve_body_re_body_msgs::StatusFT;

template <template <typename Type> class T>
class ArmStatus_Ports : public PortsContainer<Status, StatusArm > {
public:
    ArmStatus_Ports(RTT::TaskContext &tc, const std::string &prefix, StatusArm Status::*ptr);
};

template <template <typename Type> class T>
class HandStatus_Ports : public PortsContainer<Status, StatusHand > {
public:
    HandStatus_Ports(RTT::TaskContext &tc, const std::string &prefix, StatusHand Status::*ptr);
};

template <template <typename Type> class T>
class MotorStatus_Ports : public PortsContainer<Status, StatusMotor > {
public:
    MotorStatus_Ports(RTT::TaskContext &tc, const std::string &prefix, StatusMotor Status::*ptr);
};

template <template <typename Type> class T>
class FTSensorStatus_Ports : public PortsContainer<Status, StatusFT > {
public:
    FTSensorStatus_Ports(RTT::TaskContext &tc, const std::string &prefix, StatusFT Status::*ptr);
};

template <template <typename Type> class T>
class VelmaStatus_Ports : public PortsContainer<void, Status > {
public:
    typedef Status Container;
    VelmaStatus_Ports(RTT::TaskContext &tc);
};

template class VelmaStatus_Ports<RTT::InputPort >;
template class VelmaStatus_Ports<RTT::OutputPort >;

};  // namespace velma_core_cs_ve_body_interface

#endif  // __VELMA_CORE_CS_VE_BODY_STATUS_PORTS_H__

