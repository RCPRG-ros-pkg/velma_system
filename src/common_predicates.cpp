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

#include "common_predicates.h"

unsigned int getValidCommandsCount(const velma_core_cs_task_cs_msgs::Command& cmd) {
    unsigned int valid_count = 0;
    valid_count += cmd.cart_valid ? 1 : 0;
    valid_count += cmd.jnt_valid ? 1 : 0;
    return valid_count;
}

bool zeroCommandsValid(const velma_core_cs_task_cs_msgs::Command& cmd) {
    return getValidCommandsCount(cmd) == 0;
}

bool oneCommandValid(const velma_core_cs_task_cs_msgs::Command& cmd) {
    return getValidCommandsCount(cmd) == 1;
}

bool moreThanOneCommandsValid(const velma_core_cs_task_cs_msgs::Command& cmd) {
    return getValidCommandsCount(cmd) > 1;
}

bool allComponentsOk(const std::vector<RTT::TaskContext*> &components) {
    for (int i = 0; i < components.size(); ++i) {
        RTT::TaskContext::TaskState task_state = components[i]->getTaskState();
        if (task_state != RTT::TaskContext::Stopped && task_state != RTT::TaskContext::Running) {
            // so the task is in on of the states:
            // Init, PreOperational, FatalError, Exception, RunTimeError
            return false;
        }
    }
    return true;
}

bool allComponentsOk(const std::vector<RTT::TaskContext*> &components, const std::vector<std::string >& running_components_names) {
    for (int i = 0; i < components.size(); ++i) {
        RTT::TaskContext::TaskState task_state = components[i]->getTaskState();
        for (int j = 0; j < running_components_names.size(); ++j) {
            if (components[i]->getName() == running_components_names[j]) {
                // this component should be in running state
                if (task_state != RTT::TaskContext::Running) {
                    return false;
                }
            }
        }

        // anyway, the component should be in stopped or running state
        if (task_state != RTT::TaskContext::Stopped && task_state != RTT::TaskContext::Running) {
            // so the task is in on of the states:
            // Init, PreOperational, FatalError, Exception, RunTimeError
            return false;
        }
    }
    return true;
}

//using namespace velma_core_cs_ve_body_msgs;

/*
bool isNaN(double d) {
    return d != d;
}

bool isInLim(double d, double lo_lim, double hi_lim) {
    return d == d && d > lo_lim && d < hi_lim;
}

bool isCmdValid(const velma_core_cs_ve_body_msgs::Command& cmd) {

    return cmd.rTact_valid
        && cmd.tMotor_valid
        && cmd.hpMotor_valid
        && cmd.htMotor_valid
        && cmd.lArm_valid
        && cmd.rArm_valid
        && cmd.lHand_valid
        && cmd.rHand_valid
        && cmd.sc_valid
        && isCmdValid(cmd.rArm) && isCmdValid(cmd.lArm)
        && isCommandValid(cmd.rHand) && isCommandValid(cmd.lHand)
        && isCommandValidTorso(cmd.tMotor)
        && isCommandValidHeadPan(cmd.hpMotor)
        && isCommandValidHeadTilt(cmd.htMotor)
        && isCommandValidTact(cmd.rTact)
        && isCommandValidSc(cmd.sc);
}

bool isStatusValid(const velma_core_ve_body_re_body_msgs::Status &st) {
// TODO:
// barrett_hand_controller_msgs/BHPressureState rHand_p
// geometry_msgs/WrenchStamped[3] lHand_f
    return st.rArm_valid
        && st.lArm_valid
        && st.rHand_valid
        && st.lHand_valid
        && st.rFt_valid
        && st.lFt_valid
        && st.tMotor_valid
        && st.hpMotor_valid
        && st.htMotor_valid
        && st.rHand_p_valid
        && st.lHand_f_valid
        && isStatusValid(st.lArm)
        && isStatusValid(st.rArm)
        && isStatusValid(st.lHand)
        && isStatusValid(st.rHand)
        && isStatusValid(st.lFt)
        && isStatusValid(st.rFt)
        && isStatusValid(st.tMotor)
        && isStatusValid(st.hpMotor)
        && isStatusValid(st.htMotor);
}
*/

