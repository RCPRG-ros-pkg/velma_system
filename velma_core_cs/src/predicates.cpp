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

#include "velma_core_cs/master.h"

using namespace RTT;

namespace velma_core_cs_types {

static const RTT::Attribute<bool >* getBoolAttribute(const std::string& component_name, const std::string& attribute_name, const std::vector<const RTT::TaskContext*> &components) {
    for (int i = 0; i < components.size(); ++i) {
        if (components[i]->getName() == component_name) {
            const RTT::Attribute<bool >* attr = static_cast<const RTT::Attribute< bool >* >(components[i]->getAttribute(attribute_name));
            if (!attr) {
                Logger::log() << Logger::Error << "Could not get attribute \'" << attribute_name << "\' of component \'" << component_name << "\'" << Logger::endl;
            }
            return attr;
        }
    }
    Logger::log() << Logger::Error << "Could not find attribute \'" << attribute_name << "\' of component \'" << component_name << "\'" << Logger::endl;

    return NULL;
}

bool inSelfCollision( const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    //static const RTT::Attribute< bool >* inCollisionWristRight = getBoolAttribute("ColDetWrR", "inCollision", components);
    //static const RTT::Attribute< bool >* inCollisionWristLeft = getBoolAttribute("ColDetWrL", "inCollision", components);
    static const RTT::Attribute< bool >* inCollision = getBoolAttribute("ColDet", "inCollision", components);
    static const RTT::Attribute< bool >* inCollision2 = getBoolAttribute("ColDetRep", "inCollision", components);

    // Do not check collisions in wrists, because the collision avoidance algorithms are doing quite a good job
    //return inCollisionWristRight->get() || inCollisionWristLeft->get() || inCollision->get() || inCollision2->get();
    return inCollision->get() || inCollision2->get();
}

bool veBodyInSafeState( const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    return in_data->b_st.sc.safe_behavior;
}

bool veBodyStatusValid( const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    return in_data->b_st_valid && in_data->b_st.sc_valid && in_data->b_st.rArm_valid && in_data->b_st.lArm_valid && in_data->b_st.tMotor_valid && in_data->b_st.hpMotor_valid && in_data->b_st.htMotor_valid;
}

bool isCartRCmdValid(const velma_core_cs_task_cs_msgs::Command& cmd) {
    return cmd.cart_r_valid &&
                        (cmd.cart_r.imp_valid || cmd.cart_r.pose_valid || cmd.cart_r.tool_valid);

}

bool isCartLCmdValid(const velma_core_cs_task_cs_msgs::Command& cmd) {
    return cmd.cart_l_valid &&
                        (cmd.cart_l.imp_valid || cmd.cart_l.pose_valid || cmd.cart_l.tool_valid);

}

bool recvCartImpCmd( const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    return  isCartRCmdValid(in_data->cmd) ||
            isCartRCmdValid(in_data->cmd2) ||
            isCartRCmdValid(in_data->cmd3) || 
            isCartLCmdValid(in_data->cmd) ||
            isCartLCmdValid(in_data->cmd2) ||
            isCartLCmdValid(in_data->cmd3);
    /*
    bool r_cart_valid = in_data->cmd.cart_r_valid && (in_data->cmd.cart_r.imp_valid ||
                        in_data->cmd.cart_r.pose_valid || in_data->cmd.cart_r.tool_valid);
    bool r_cart2_valid = in_data->cmd2.cart_r_valid &&
                        (in_data->cmd2.cart_r.imp_valid ||
                        in_data->cmd2.cart_r.pose_valid ||
                        in_data->cmd2.cart_r.tool_valid);

    bool l_cart_valid = in_data->cmd.cart_l_valid && (in_data->cmd.cart_l.imp_valid ||
                        in_data->cmd.cart_l.pose_valid || in_data->cmd.cart_l.tool_valid);
    bool l_cart2_valid = in_data->cmd2.cart_l_valid &&
                        (in_data->cmd2.cart_l.imp_valid ||
                        in_data->cmd2.cart_l.pose_valid ||
                        in_data->cmd2.cart_l.tool_valid);
    return r_cart_valid || r_cart2_valid || l_cart_valid || l_cart2_valid;
    */
}

bool recvJntImpCmd( const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    return in_data->cmd.jnt_valid || in_data->cmd2.jnt_valid || in_data->cmd3.jnt_valid;
}

bool recvRelaxCmd( const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    return in_data->cmd.relax_valid && in_data->cmd.relax;
}

bool recvOneCmd( const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    unsigned int valid_count = 0;
    valid_count += (recvCartImpCmd(in_data, components) ? 1 : 0);
    valid_count += (recvJntImpCmd(in_data, components) ? 1 : 0);
    valid_count += (recvRelaxCmd(in_data, components) ? 1 : 0);
    return valid_count == 1;
}

bool motorsReady( const InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*> &components) {
    return  in_data->b_st.tMotor.homing_required_valid && (!in_data->b_st.tMotor.homing_required) && in_data->b_st.tMotor.enabled &&
            in_data->b_st.htMotor.homing_required_valid && (!in_data->b_st.htMotor.homing_required) && in_data->b_st.htMotor.enabled &&
            in_data->b_st.hpMotor.homing_required_valid && (!in_data->b_st.hpMotor.homing_required) && in_data->b_st.hpMotor.enabled;
}

};  // namespace velma_core_cs_types

REGISTER_PREDICATE( velma_core_cs_types::inSelfCollision );
REGISTER_PREDICATE( velma_core_cs_types::veBodyInSafeState );
REGISTER_PREDICATE( velma_core_cs_types::veBodyStatusValid );
REGISTER_PREDICATE( velma_core_cs_types::recvCartImpCmd );
REGISTER_PREDICATE( velma_core_cs_types::recvJntImpCmd );
REGISTER_PREDICATE( velma_core_cs_types::recvRelaxCmd );
REGISTER_PREDICATE( velma_core_cs_types::recvOneCmd );
REGISTER_PREDICATE( velma_core_cs_types::motorsReady );
