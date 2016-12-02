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

#include <rtt/Component.hpp>

#include "common_interfaces/interface_tx.h"
#include "common_interfaces/interface_rx.h"

#include "common_interfaces/message_split.h"
#include "common_interfaces/message_concate.h"

//#include "velma_core_cs_ve_body_interface/command_ports.h"
#include "velma_core_cs_ve_body_interface/port_data.h"

#include "velma_core_cs_ve_body_msgs/Status.h"
#include "velma_core_cs_ve_body_msgs/Command.h"

using namespace velma_core_cs_ve_body_msgs;

typedef InterfaceTx<velma_core_cs_ve_body_msgs::Status > VelmaCoreCsVeBodyStatusTx;
ORO_LIST_COMPONENT_TYPE(VelmaCoreCsVeBodyStatusTx)

typedef InterfaceRx<velma_core_cs_ve_body_msgs::Status > VelmaCoreCsVeBodyStatusRx;
ORO_LIST_COMPONENT_TYPE(VelmaCoreCsVeBodyStatusRx)

typedef InterfaceTx<velma_core_cs_ve_body_msgs::Command > VelmaCoreCsVeBodyCommandTx;
ORO_LIST_COMPONENT_TYPE(VelmaCoreCsVeBodyCommandTx)

typedef InterfaceRx<velma_core_cs_ve_body_msgs::Command > VelmaCoreCsVeBodyCommandRx;
ORO_LIST_COMPONENT_TYPE(VelmaCoreCsVeBodyCommandRx)

typedef MessageSplit<Command_Ports > VelmaCoreCsVeBodyCommandSplit;
ORO_LIST_COMPONENT_TYPE(VelmaCoreCsVeBodyCommandSplit)

typedef MessageConcate<Status_Ports > VelmaCoreCsVeBodyStatusConcate;
ORO_LIST_COMPONENT_TYPE(VelmaCoreCsVeBodyStatusConcate)

typedef MessageSplit<Status_Ports > VelmaCoreCsVeBodyStatusSplit;
ORO_LIST_COMPONENT_TYPE(VelmaCoreCsVeBodyStatusSplit)

typedef MessageConcate<Command_Ports > VelmaCoreCsVeBodyCommandConcate;
ORO_LIST_COMPONENT_TYPE(VelmaCoreCsVeBodyCommandConcate)

ORO_CREATE_COMPONENT_LIBRARY()
