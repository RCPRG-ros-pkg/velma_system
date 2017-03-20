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

namespace velma_core_cs_types {

bool graphSafe( const InputDataConstPtr& in_data, const std::vector<RTT::TaskContext*> &components) {
    for (int i = 0; i < components.size(); ++i) {
        const std::string& name = components[i]->getName();
        RTT::TaskContext::TaskState state = components[i]->getTaskState();

        if (state != RTT::TaskContext::Running) {
            if (name == "safe" || name == "Mass" || name == "FK" || name == "JntImp") {
                return false;
            }
        }
        
        if (state != RTT::TaskContext::Stopped && state != RTT::TaskContext::Running) {
            return false;
        }
    }
    return true;
}

bool graphCartImp( const InputDataConstPtr& in_data, const std::vector<RTT::TaskContext*> &components) {
    for (int i = 0; i < components.size(); ++i) {
        const std::string& name = components[i]->getName();
        RTT::TaskContext::TaskState state = components[i]->getTaskState();

        if (state != RTT::TaskContext::Running) {
            if (name == "CImp" || name == "Mass" || name == "JntLimit" || name == "PoseIntLeft" || name == "PoseIntRight" || name == "FK") {
                return false;
            }
        }
        
        if (state != RTT::TaskContext::Stopped && state != RTT::TaskContext::Running) {
            return false;
        }
    }
    return true;
}

bool graphJntImp( const InputDataConstPtr& in_data, const std::vector<RTT::TaskContext*> &components) {
    for (int i = 0; i < components.size(); ++i) {
        const std::string& name = components[i]->getName();
        RTT::TaskContext::TaskState state = components[i]->getTaskState();

        if (state != RTT::TaskContext::Running) {
            if (name == "TrajectoryGeneratorJoint" || name == "Mass" || name == "JntImp" || name == "FK") {
                return false;
            }
        }
        
        if (state != RTT::TaskContext::Stopped && state != RTT::TaskContext::Running) {
            return false;
        }
    }
    return true;
}

bool veBodyInSafeState( const InputDataConstPtr& in_data, const std::vector<RTT::TaskContext*> &components) {
    return in_data->b_st.sc.safe_behavior;
}

bool veBodyStatusValid( const InputDataConstPtr& in_data, const std::vector<RTT::TaskContext*> &components) {
    return in_data->b_st.sc_valid && in_data->b_st.rArm_valid && in_data->b_st.lArm_valid && in_data->b_st.tMotor_valid && in_data->b_st.hpMotor_valid && in_data->b_st.htMotor_valid;
}

bool recvCartImpCmd( const InputDataConstPtr& in_data, const std::vector<RTT::TaskContext*> &components) {
    return in_data->cmd.cart_r.imp_valid || in_data->cmd.cart_r.pose_valid || in_data->cmd.cart_r.tool_valid ||
           in_data->cmd.cart_l.imp_valid || in_data->cmd.cart_l.pose_valid || in_data->cmd.cart_l.tool_valid;
}

bool recvJntImpCmd( const InputDataConstPtr& in_data, const std::vector<RTT::TaskContext*> &components) {
    return in_data->cmd.jnt_valid;
}

bool recvOneCmd( const InputDataConstPtr& in_data, const std::vector<RTT::TaskContext*> &components) {
    unsigned int valid_count = 0;
    valid_count += (recvCartImpCmd(in_data, components) ? 1 : 0);
    valid_count += (recvJntImpCmd(in_data, components) ? 1 : 0);
    return valid_count == 1;
}

};  // namespace velma_core_cs_types

REGISTER_PREDICATE( velma_core_cs_types::graphSafe );
REGISTER_PREDICATE( velma_core_cs_types::graphCartImp );
REGISTER_PREDICATE( velma_core_cs_types::graphJntImp );
REGISTER_PREDICATE( velma_core_cs_types::veBodyInSafeState );
REGISTER_PREDICATE( velma_core_cs_types::veBodyStatusValid );
REGISTER_PREDICATE( velma_core_cs_types::recvCartImpCmd );
REGISTER_PREDICATE( velma_core_cs_types::recvJntImpCmd );
REGISTER_PREDICATE( velma_core_cs_types::recvOneCmd );

