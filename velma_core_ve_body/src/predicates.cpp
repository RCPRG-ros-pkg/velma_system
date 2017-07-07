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

static const RTT::Attribute<bool >* getBoolAttribute(const std::string& component_name, const std::string& attribute_name, const std::vector<const RTT::TaskContext*> &components) {
    for (int i = 0; i < components.size(); ++i) {
        if (components[i]->getName() == component_name) {
            const RTT::Attribute<bool >* attr = static_cast<const RTT::Attribute< bool >* >(components[i]->getAttribute(attribute_name));
            if (!attr) {
                RTT::log(RTT::Error) << "Could not get attribute \'" << attribute_name << "\' of component \'" << component_name << "\'" << RTT::endlog();
            }
            return attr;
        }
    }
    RTT::log(RTT::Error) << "Could not find attribute \'" << attribute_name << "\' of component \'" << component_name << "\'" << RTT::endlog();

    return NULL;
}

bool safeIterationsPassed500(const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    static const RTT::Attribute< bool >* safeIterationsOver500_attrib = getBoolAttribute("safe", "safeIterationsOver500", components);
    return safeIterationsOver500_attrib->get();
}

bool cmdExitSafeState( const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    return in_data->hi_cmd.sc_valid && (in_data->hi_cmd.sc == 1);
}

bool rLwrOk( const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    static const RTT::Attribute< bool >* rLwrOk_attrib = getBoolAttribute("hw_state", "rLwrOk", components);
    return rLwrOk_attrib->get();
}

bool rLwrInCmdState( const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    static const RTT::Attribute< bool >* rLwrCmdState_attrib = getBoolAttribute("hw_state", "rLwrCmdState", components);
    return rLwrCmdState_attrib->get();
}

bool lLwrOk( const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    static const RTT::Attribute< bool >* lLwrOk_attrib = getBoolAttribute("hw_state", "lLwrOk", components);
    return lLwrOk_attrib->get();
}

bool lLwrInCmdState( const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    static const RTT::Attribute< bool >* lLwrCmdState_attrib = getBoolAttribute("hw_state", "lLwrCmdState", components);
    return lLwrCmdState_attrib->get();
}

bool tMotorOk( const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    return in_data->lo_ec_st_valid && in_data->lo_ec_st.TorsoPan.Inputs.Positionactualvalue_valid;
}

bool hpMotorOk( const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    return in_data->lo_ec_st_valid && in_data->lo_ec_st.HeadPan.Inputs.Positionactualvalue_valid;
}

bool htMotorOk( const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    return in_data->lo_ec_st_valid && in_data->lo_ec_st.HeadTilt.Inputs.Positionactualvalue_valid;
}

bool rLwrCmdOk( const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    return in_data->hi_cmd.rArm_valid && isCmdArmValid(in_data->hi_cmd.rArm);
}

bool lLwrCmdOk( const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    return in_data->hi_cmd.lArm_valid && isCmdArmValid(in_data->hi_cmd.lArm);
}

bool tCmdOk( const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    return in_data->hi_cmd.tMotor_i_valid && isCmdTorsoValid(in_data->hi_cmd.tMotor_i);
}

bool hpCmdOk( const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    return in_data->hi_cmd.hpMotor_valid && in_data->hi_cmd.hpMotor.q_valid;// && isCmdHeadPanValid(...);   // TODO
}

bool htCmdOk( const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    return in_data->hi_cmd.htMotor_valid && in_data->hi_cmd.htMotor.q_valid;// && isCmdHeadPanValid(...);   // TODO
}

bool recvStatus( const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    return in_data->lo_ec_st_valid;
}

bool recvCommand( const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    return in_data->hi_cmd_valid;
}

};  // namespace velma_core_ve_body_types

REGISTER_PREDICATE( velma_core_ve_body_types::safeIterationsPassed500 );
REGISTER_PREDICATE( velma_core_ve_body_types::cmdExitSafeState );
REGISTER_PREDICATE( velma_core_ve_body_types::rLwrOk );
REGISTER_PREDICATE( velma_core_ve_body_types::rLwrInCmdState );
REGISTER_PREDICATE( velma_core_ve_body_types::lLwrOk );
REGISTER_PREDICATE( velma_core_ve_body_types::lLwrInCmdState );
REGISTER_PREDICATE( velma_core_ve_body_types::tMotorOk );
REGISTER_PREDICATE( velma_core_ve_body_types::hpMotorOk );
REGISTER_PREDICATE( velma_core_ve_body_types::htMotorOk );
REGISTER_PREDICATE( velma_core_ve_body_types::rLwrCmdOk );
REGISTER_PREDICATE( velma_core_ve_body_types::lLwrCmdOk );
REGISTER_PREDICATE( velma_core_ve_body_types::tCmdOk );
REGISTER_PREDICATE( velma_core_ve_body_types::hpCmdOk );
REGISTER_PREDICATE( velma_core_ve_body_types::htCmdOk );
REGISTER_PREDICATE( velma_core_ve_body_types::recvStatus );
REGISTER_PREDICATE( velma_core_ve_body_types::recvCommand );

