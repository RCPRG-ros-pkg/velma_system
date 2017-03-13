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

#include "velma_core_ve_body/master.h"
#include "common_predicates.h"

namespace velma_core_ve_body_types {

class BehaviorSafe : public BehaviorBase {
public:
    BehaviorSafe() :
        BehaviorBase("behavior_velma_core_ve_body_safe", "safe")
    {
        addRunningComponent("safe");
    }

    bool checkErrorCondition(
                const boost::shared_ptr<InputData >& in_data,
                const std::vector<RTT::TaskContext*> &components,
                ErrorCausePtr result) const
    {
        return false;
    }

    bool checkStopCondition(
                const boost::shared_ptr<InputData >& in_data,
                const std::vector<RTT::TaskContext*> &components) const
    {
        bool rLwrOk = isLwrOk(in_data->st.rArmFriRobot, in_data->st.rArmFriIntf);
        bool lLwrOk = isLwrOk(in_data->st.lArmFriRobot, in_data->st.lArmFriIntf);
        bool rLwrCmd = isLwrInCmdState(in_data->st.rArmFriIntf);
        bool lLwrCmd = isLwrInCmdState(in_data->st.lArmFriIntf);
        bool hwOk = (rLwrOk && lLwrOk && rLwrCmd && lLwrCmd);

        bool resetCmd = (in_data->cmd.sc_valid && in_data->cmd.sc.cmd == 1);

//        std::cout << (hwOk?"t":"f") << (resetCmd?"t":"f") << (isCmdValid(in_data->cmd)?"t":"f") << (isStatusValid(in_data->st)?"t":"f") << std::endl;
        if (hwOk && resetCmd && isCmdValid(in_data->cmd) && isStatusValid(in_data->st))
        {
            return true;
        }
        return false;
    }
};

class StateSafe : public StateBase {
public:
    StateSafe() :
        StateBase("state_velma_core_ve_body_safe", "safe", "behavior_velma_core_ve_body_safe")
    {
    }

    virtual bool checkInitialCondition(
                const boost::shared_ptr<InputData >& in_data,
                const std::vector<RTT::TaskContext*> &components,
                const std::string& prev_state_name,
                bool in_error) const
    {
        if (prev_state_name == "state_velma_core_ve_body_safe") {
            return false;
        }

        return true;
    }
};

};  // namespace velma_core_ve_body_types

REGISTER_BEHAVIOR( velma_core_ve_body_types::BehaviorSafe );

REGISTER_STATE( velma_core_ve_body_types::StateSafe );

