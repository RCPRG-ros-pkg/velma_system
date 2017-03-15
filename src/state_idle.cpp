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
#include "common_predicates.h"

namespace velma_core_cs_types {

class BehaviorIdle : public BehaviorBase {
public:
    BehaviorIdle() :
        BehaviorBase("behavior_velma_core_cs_idle", "idle")
    {
        addRunningComponent("idle");
    }

    bool checkErrorCondition(
                const boost::shared_ptr<InputData >& in_data,
                const std::vector<RTT::TaskContext*> &components,
                ErrorCausePtr result) const
    {
        // this behavior has no error condition

        return false;
    }

    bool checkStopCondition(
                const boost::shared_ptr<InputData >& in_data,
                const std::vector<RTT::TaskContext*> &components) const
    {
        // this behavior is stopped only if lower subsystem is operational (has no error condition)
        // and started to listen this level
        if (in_data->b_st.sc_valid == true && in_data->b_st.sc.error == false && in_data->b_st.sc.safe_behavior == false &&
            in_data->b_st.rArm_valid && in_data->b_st.lArm_valid && in_data->b_st.tMotor_valid) {
            return true;
        }

        return false;
    }
};

class StateIdle : public StateBase {
public:
    StateIdle() :
        StateBase("state_velma_core_cs_idle", "idle", "behavior_velma_core_cs_idle")
    {
    }

    bool checkInitialCondition(
                const boost::shared_ptr<InputData >& in_data,
                const std::vector<RTT::TaskContext*> &components,
                const std::string& prev_state_name,
                bool in_error) const
    {
        if (prev_state_name == "state_velma_core_cs_idle") {
            return false;
        }

        // this behavior is started only if lower subsystem is not operational
        // and is not listening this level
        if (in_data->b_st.sc.safe_behavior == true ||
            !in_data->b_st.sc_valid || !in_data->b_st.rArm_valid || !in_data->b_st.lArm_valid || !in_data->b_st.tMotor_valid) {
            return true;
        }

        return false;
    }
};

};  // namespace velma_core_cs_types

REGISTER_BEHAVIOR( velma_core_cs_types::BehaviorIdle );
REGISTER_STATE( velma_core_cs_types::StateIdle );

