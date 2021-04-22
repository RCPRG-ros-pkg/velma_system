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

using namespace velma_core_ve_body_types;

bool isLwrOk(const lwr_msgs::FriRobotState& friRobot, const lwr_msgs::FriIntfState& friIntf) {
    if (friRobot.power != 0x7F                           // error
        || friRobot.error != 0                           // error
        || friRobot.warning != 0                         // TODO: check if this is error
        || friRobot.control != lwr_msgs::FriRobotState::FRI_CTRL_JNT_IMP        // error
        || friIntf.quality <= lwr_msgs::FriIntfState::FRI_QUALITY_UNACCEPTABLE) // error
    {
        return false;
    }
    return true;
}

bool isLwrInCmdState(const lwr_msgs::FriIntfState& friIntf) {
    return friIntf.state == lwr_msgs::FriIntfState::FRI_STATE_CMD;
}

bool isNaN(double d) {
    return d != d;
}

bool isInLim(double d, double lo_lim, double hi_lim) {
    return d == d && d > lo_lim && d < hi_lim;
}

//
// command validation
//
/*
bool isCommandValidTorso(const velma_core_ve_body_re_body_msgs::CommandMotor &cmd) {
// TODO
    return true;
}

bool isCommandValidHeadPan(const velma_core_ve_body_re_body_msgs::CommandMotor &cmd) {
// TODO
    return true;
}

bool isCommandValidHeadTilt(const velma_core_ve_body_re_body_msgs::CommandMotor &cmd) {
// TODO
    return true;
}

bool isCommandValid(const barrett_hand_msgs::CommandHand &cmd) {
// TODO
    return true;
}

bool isCommandValidTact(const velma_core_ve_body_re_body_msgs::CommandSimple &cmd) {
// TODO
    return true;
}

bool isCommandValidSc(const velma_core_ve_body_re_body_msgs::CommandSimple &cmd) {
// TODO
    return true;
}

*/

bool isCmdArmValid(const velma_core_cs_ve_body_msgs::CommandArm& cmd) {
    // Standard limits:
    double arm_t_limits[7] = {50.0, 50.0, 30.0, 20.0, 20.0, 10.0, 10.0};

    // These are limits for grasping heavy objects in experiments for gravity compensation:
    //double arm_t_limits[7] = {100.0, 100.0, 60.0, 50.0, 40.0, 20.0, 20.0};

    for (int i = 0; i < cmd.t.size(); ++i) {
        if (!isInLim(cmd.t[i], -arm_t_limits[i], arm_t_limits[i])) {
            return false;
        }
    }
    return true;
}

bool isCmdTorsoValid(double cmd_tMotor_t) {
    double tMotor_t_limit = 100;
    if (!isInLim(cmd_tMotor_t, -tMotor_t_limit, tMotor_t_limit)) {
        return false;
    }
    return true;
}

//
// status validation
//

bool isStatusValid(const velma_core_cs_ve_body_msgs::StatusArm &st) {
    double arm_q_limits_lo[7] = {-2.96, -2.09, -2.96, -2.09, -2.96, -2.09, -2.96};
    double arm_q_limits_hi[7] = {2.96, 2.09, 2.96, 2.09, 2.96, 2.09, 2.96};

// TODO: double arm_dq_limits[7] = {2, 2, 2, 2, 2, 2, 2}

    for (int i = 0; i < st.q.size(); ++i) {
        if (!isInLim(st.q[i], arm_q_limits_lo[i], arm_q_limits_hi[i]) ||
            isNaN(st.dq[i]) ||
            isNaN(st.t[i]) ||
            isNaN(st.gt[i]))
        {
//            if (err) {
//                err->setBit(LWR_NAN_LIM_bit, true);
//            }
            return false;
        }
    }

// TODO:
//    isInLim(st.w.force.x, 
//    st.mmx
    return true;
}

//bool isStatusValid(const barrett_hand_msgs::StatusHand &st) {
// TODO
//    return true;
//}

bool isStatusValid(const velma_core_cs_ve_body_msgs::Status &st) {
    bool status_valid = st.rArm_valid
        && st.lArm_valid
        && st.rFt_valid
        && st.lFt_valid
        && st.tMotor_valid;
//        && st.hpMotor_valid
//        && st.htMotor_valid;

    if (!status_valid) {
        return false;
    }

    if (!isStatusValid(st.rArm)) {
        return false;
    }

    if (!isStatusValid(st.lArm)) {
        return false;
    }

// TODO:
    return true;
//        && isStatusValid(st.lFt, err)
//        && isStatusValid(st.rFt, err)
//        && isStatusValid(st.tMotor, err)
//        && isStatusValid(st.hpMotor, err)
//        && isStatusValid(st.htMotor, err);
}

