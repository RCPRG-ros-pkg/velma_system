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

#include <rtt/Logger.hpp>
#include "velma_core_ve_body/master.h"
#include "common_predicates.h"

namespace velma_core_ve_body_types {

static RTT::OperationCaller<uint32_t()> getOperationSafeIterationsPassed(const std::vector<RTT::TaskContext*> &components) {
    RTT::OperationCaller<uint32_t()> safeIterationsPassed;
    for (int i = 0; i < components.size(); ++i) {
        if (components[i]->getName() == "safe") {
            safeIterationsPassed = components[i]->getOperation("safeIterationsPassed");
            break;
        }
    }

    if (!safeIterationsPassed.ready()) {
        RTT::Logger::log() << RTT::Logger::Error << "could not get operation safeIterationsPassed" << RTT::Logger::endl;
    }
    return safeIterationsPassed;
}

bool safeIterationsPassed500(const InputDataConstPtr& in_data, const std::vector<RTT::TaskContext*> &components) {
    static RTT::OperationCaller<uint32_t()> safeIterationsPassed = getOperationSafeIterationsPassed(components);

    if (safeIterationsPassed.ready()) {
        return (safeIterationsPassed() > 500);
    }
    return false;
}

bool rLwrOk( const InputDataConstPtr& in_data, const std::vector<RTT::TaskContext*> &components) {
    return isLwrOk(in_data->lo_st.rArmFriRobot, in_data->lo_st.rArmFriIntf);
}

bool rLwrInCmdState( const InputDataConstPtr& in_data, const std::vector<RTT::TaskContext*> &components) {
    return isLwrInCmdState(in_data->lo_st.rArmFriIntf);
}

bool lLwrOk( const InputDataConstPtr& in_data, const std::vector<RTT::TaskContext*> &components) {
    return isLwrOk(in_data->lo_st.lArmFriRobot, in_data->lo_st.lArmFriIntf);
}

bool lLwrInCmdState( const InputDataConstPtr& in_data, const std::vector<RTT::TaskContext*> &components) {
    return isLwrInCmdState(in_data->lo_st.lArmFriIntf);
}

bool rLwrCmdOk( const InputDataConstPtr& in_data, const std::vector<RTT::TaskContext*> &components) {
    return in_data->hi_cmd.rArm_valid && isCmdArmValid(in_data->hi_cmd.rArm);
}

bool lLwrCmdOk( const InputDataConstPtr& in_data, const std::vector<RTT::TaskContext*> &components) {
    return in_data->hi_cmd.lArm_valid && isCmdArmValid(in_data->hi_cmd.lArm);
}

bool tCmdOk( const InputDataConstPtr& in_data, const std::vector<RTT::TaskContext*> &components) {
    return in_data->hi_cmd.tMotor_i_valid && isCmdTorsoValid(in_data->hi_cmd.tMotor_i);
}

bool cmdExitSafeState( const InputDataConstPtr& in_data, const std::vector<RTT::TaskContext*> &components) {
    return in_data->hi_cmd.sc_valid && (in_data->hi_cmd.sc == 1);
}

};  // namespace velma_core_ve_body_types

REGISTER_PREDICATE( velma_core_ve_body_types::safeIterationsPassed500 );
REGISTER_PREDICATE( velma_core_ve_body_types::rLwrOk );
REGISTER_PREDICATE( velma_core_ve_body_types::rLwrInCmdState );
REGISTER_PREDICATE( velma_core_ve_body_types::lLwrOk );
REGISTER_PREDICATE( velma_core_ve_body_types::lLwrInCmdState );
REGISTER_PREDICATE( velma_core_ve_body_types::rLwrCmdOk );
REGISTER_PREDICATE( velma_core_ve_body_types::lLwrCmdOk );
REGISTER_PREDICATE( velma_core_ve_body_types::tCmdOk );
REGISTER_PREDICATE( velma_core_ve_body_types::cmdExitSafeState );

