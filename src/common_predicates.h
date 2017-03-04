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

#ifndef VELMA_CORE_VE_BODY_COMMON_PREDICATES_H__
#define VELMA_CORE_VE_BODY_COMMON_PREDICATES_H__

#include "velma_core_cs_ve_body_msgs/Command.h"
#include "velma_core_ve_body_re_body_msgs/Status.h"
#include "abstract_behavior.h"

bool isLwrOk(const velma_core_ve_body_re_body_msgs::StatusArmFriRobot& friRobot, const velma_core_ve_body_re_body_msgs::StatusArmFriIntf& friIntf);
bool isLwrInCmdState(const velma_core_ve_body_re_body_msgs::StatusArmFriIntf& friIntf);
bool isNaN(double d);
bool isInLim(double d, double lo_lim, double hi_lim);
bool isCmdValid(const velma_core_cs_ve_body_msgs::Command& cmd, velma_core_ve_body_types::ErrorCausePtr err = velma_core_ve_body_types::ErrorCausePtr());
//bool isCmdValid(const velma_core_cs_ve_body_msgs::Command& cmd);
bool isStatusValid(const velma_core_ve_body_re_body_msgs::Status &st, velma_core_ve_body_types::ErrorCausePtr err = velma_core_ve_body_types::ErrorCausePtr());
std::string getErrorReasonStr(velma_core_ve_body_types::ErrorCauseConstPtr err);

#endif  // VELMA_CORE_VE_BODY_COMMON_PREDICATES_H__

