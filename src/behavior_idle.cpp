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

#include "common_behavior/abstract_behavior.h"
#include "input_data.h"
#include "common_predicates.h"

namespace velma_core_ve_body_types {

class BehaviorIdle : public common_behavior::BehaviorBase {
public:
    BehaviorIdle() :
        common_behavior::BehaviorBase("behavior_velma_core_ve_body_idle")
    {
        addRunningComponent("bypass");
    }

    bool checkErrorCondition(
                const common_behavior::InputData& in_data,
                const std::vector<RTT::TaskContext*> &components) const
    {
        const InputData& in = static_cast<const InputData& >(in_data);
        bool rLwrOk = isLwrOk(in.status_.rArmFriRobot, in.status_.rArmFriIntf);
        bool lLwrOk = isLwrOk(in.status_.lArmFriRobot, in.status_.lArmFriIntf);
        bool rLwrCmd = isLwrInCmdState(in.status_.rArmFriIntf);
        bool lLwrCmd = isLwrInCmdState(in.status_.lArmFriIntf);
        bool hwOk = (rLwrOk && lLwrOk && rLwrCmd && lLwrCmd);

        if (hwOk && isCmdValid(in.cmd_) && isStatusValid(in.status_))
        {
            return false;
        }

        return true;
    }

    bool checkStopCondition(
                const common_behavior::InputData& in_data,
                const std::vector<RTT::TaskContext*> &components) const
    {
        const InputData& in = static_cast<const InputData& >(in_data);
        return false;
    }
};

};  // namespace velma_core_ve_body_types

REGISTER_BEHAVIOR( velma_core_ve_body_types::BehaviorIdle );

