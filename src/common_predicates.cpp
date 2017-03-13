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
#include <kuka_lwr_fri/friComm.h>

using namespace velma_core_ve_body_types;

bool isLwrOk(const velma_core_ve_body_re_body_msgs::StatusArmFriRobot& friRobot, const velma_core_ve_body_re_body_msgs::StatusArmFriIntf& friIntf) {
    if (friRobot.power != 0x7F                           // error
        || friRobot.error != 0                           // error
        || friRobot.warning != 0                         // TODO: check if this is error
        || friRobot.control != FRI_CTRL_JNT_IMP          // error
        || friIntf.quality <= FRI_QUALITY_UNACCEPTABLE)    // error
    {
        return false;
    }
    return true;
}

bool isLwrInCmdState(const velma_core_ve_body_re_body_msgs::StatusArmFriIntf& friIntf) {
    return friIntf.state == FRI_STATE_CMD;
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

bool isCommandValid(const velma_core_ve_body_re_body_msgs::CommandHand &cmd) {
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

bool isCmdValid(const velma_core_ve_body_re_body_msgs::CommandArm& cmd) {
    double arm_t_limits[7] = {100.0, 100.0, 100.0, 100.0, 100.0, 60.0, 60.0};
    for (int i = 0; i < cmd.t.size(); ++i) {
        if (!isInLim(cmd.t[i], -arm_t_limits[i], arm_t_limits[i])) {
            return false;
        }
    }
    return true;
}

bool isCmdValid(const velma_core_cs_ve_body_msgs::Command& cmd) {

    return cmd.tMotor_valid
        && cmd.hpMotor_valid
        && cmd.htMotor_valid
        && cmd.lArm_valid
        && cmd.rArm_valid
        && isCmdValid(cmd.rArm) && isCmdValid(cmd.lArm)
        && isCommandValidTorso(cmd.tMotor)
        && isCommandValidHeadPan(cmd.hpMotor)
        && isCommandValidHeadTilt(cmd.htMotor);
}
*/
bool isCmdValid(const velma_core_cs_ve_body_msgs::Command& cmd, ErrorCausePtr err) {
    if (err) {
        err->setBit(CMD_T_MOTOR_INVALID_bit, !cmd.tMotor_valid);
        err->setBit(CMD_T_MOTOR_INVALID_bit, !cmd.tMotor.i_valid);
//        err->setBit(CMD_HP_MOTOR_INVALID_bit, !cmd.hpMotor_valid);
//        err->setBit(CMD_HT_MOTOR_INVALID_bit, !cmd.htMotor_valid);
        err->setBit(CMD_L_ARM_INVALID_bit, !cmd.lArm_valid);
        err->setBit(CMD_R_ARM_INVALID_bit, !cmd.rArm_valid);
    }

    if (!cmd.tMotor_valid ||
        // !cmd.hpMotor_valid || !cmd.htMotor_valid ||
         !cmd.lArm_valid || !cmd.rArm_valid) {
        return false;
    }

    double arm_t_limits[7] = {100.0, 100.0, 100.0, 100.0, 100.0, 60.0, 60.0};

    if (err) {
        for (int i = 0; i < cmd.rArm.t.size(); ++i) {
            if (isNaN(cmd.rArm.t[i])) {
                err->setBit(CMD_R_ARM_NAN_bit, true);
            }
            else if (!isInLim(cmd.rArm.t[i], -arm_t_limits[i], arm_t_limits[i])) {
                err->setBit(CMD_R_ARM_LIM_bit, true);
            }
        }

        for (int i = 0; i < cmd.lArm.t.size(); ++i) {
            if (isNaN(cmd.lArm.t[i])) {
                err->setBit(CMD_L_ARM_NAN_bit, true);
            }
            else if (!isInLim(cmd.lArm.t[i], -arm_t_limits[i], arm_t_limits[i])) {
                err->setBit(CMD_L_ARM_LIM_bit, true);
            }
        }
        err->setBit(CMD_T_MOTOR_T_NAN_bit, isNaN(cmd.tMotor.i));
        if (err->orValue()) {
            return false;
        }
    }
    else {
        for (int i = 0; i < cmd.rArm.t.size(); ++i) {
            if (!isInLim(cmd.rArm.t[i], -arm_t_limits[i], arm_t_limits[i])) {
                return false;
            }
        }

        for (int i = 0; i < cmd.lArm.t.size(); ++i) {
            if (!isInLim(cmd.lArm.t[i], -arm_t_limits[i], arm_t_limits[i])) {
                return false;
            }
        }
    }

    double tMotor_i_limit = 1000;
    if (!isInLim(cmd.tMotor.i, -tMotor_i_limit, tMotor_i_limit)) {
        return false;
    }

//    double hpMotor_dq_limit = 1.0;
//    if (!isInLim(cmd.hpMotor.dq, -hpMotor_dq_limit, hpMotor_dq_limit)) {
//        return false;
//    }

//    double htMotor_dq_limit = 1.0;
//    if (!isInLim(cmd.htMotor.dq, -htMotor_dq_limit, htMotor_dq_limit)) {
//        return false;
//    }

    return true;
}


//
// status validation
//

bool isStatusValid(const velma_core_ve_body_re_body_msgs::StatusArm &st, ErrorCausePtr err=ErrorCausePtr()) {
    double arm_q_limits_lo[7] = {-2.96, -2.09, -2.96, -2.09, -2.96, -2.09, -2.96};
    double arm_q_limits_hi[7] = {2.96, 2.09, 2.96, 2.09, 2.96, 2.09, 2.96};

// TODO: double arm_dq_limits[7] = {2, 2, 2, 2, 2, 2, 2}

    for (int i = 0; i < st.q.size(); ++i) {
        if (!isInLim(st.q[i], arm_q_limits_lo[i], arm_q_limits_hi[i]) ||
            isNaN(st.dq[i]) ||
            isNaN(st.t[i]) ||
            isNaN(st.gt[i]))
        {
            if (err) {
                err->setBit(LWR_NAN_LIM_bit, true);
            }
            return false;
        }
    }

// TODO:
//    isInLim(st.w.force.x, 
//    st.mmx
    return true;
}

//bool isStatusValid(const velma_core_ve_body_re_body_msgs::StatusHand &st) {
// TODO
//    return true;
//}

bool isStatusValid(const velma_core_ve_body_re_body_msgs::StatusMotor &st, ErrorCausePtr err=ErrorCausePtr()) {
// TODO
    return true;
}

bool isStatusValid(const velma_core_ve_body_re_body_msgs::StatusFT &st, ErrorCausePtr err=ErrorCausePtr()) {
// TODO
    return true;
}

bool isStatusValid(const velma_core_ve_body_re_body_msgs::Status &st, ErrorCausePtr err) {
    bool status_valid = st.rArm_valid
        && st.lArm_valid
        && st.rFt_valid
        && st.lFt_valid
        && st.tMotor_valid;
//        && st.hpMotor_valid
//        && st.htMotor_valid;

    if (!status_valid) {
        if (err) {
            err->setBit(STATUS_R_LWR_INVALID_bit, !st.rArm_valid);
            err->setBit(STATUS_L_LWR_INVALID_bit, !st.lArm_valid);
            err->setBit(STATUS_R_FT_INVALID_bit, !st.rFt_valid);
            err->setBit(STATUS_L_FT_INVALID_bit, !st.lFt_valid);
            err->setBit(STATUS_T_MOTOR_INVALID_bit, !st.tMotor_valid);
//            err->setBit(STATUS_HP_MOTOR_INVALID_bit, !st.hpMotor_valid);
//            err->setBit(STATUS_HT_MOTOR_INVALID_bit, !st.htMotor_valid);
        }
        return false;
    }

    if (!isStatusValid(st.rArm, err)) {
        if (err) {
            err->setBit(STATUS_R_LWR_INVALID_bit, true);
        }
        return false;
    }

    if (!isStatusValid(st.lArm, err)) {
        if (err) {
            err->setBit(STATUS_L_LWR_INVALID_bit, true);
        }
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

std::string getErrorReasonStr(ErrorCauseConstPtr err) {
    std::string result;
    result += (err->getBit(R_LWR_bit)?"R_LWR ":"");
    result += (err->getBit(L_LWR_bit)?"L_LWR ":"");
    result += (err->getBit(R_LWR_CMD_bit)?"R_LWR_CMD ":"");
    result += (err->getBit(L_LWR_CMD_bit)?"L_LWR_CMD ":"");
    result += (err->getBit(STATUS_bit)?"STATUS ":"");
    result += (err->getBit(STATUS_R_LWR_INVALID_bit)?"ST_R_LWR_INV ":"");
    result += (err->getBit(STATUS_L_LWR_INVALID_bit)?"ST_L_LWR_INV ":"");
    result += (err->getBit(STATUS_R_FT_INVALID_bit)?"ST_R_FT_INV ":"");
    result += (err->getBit(STATUS_L_FT_INVALID_bit)?"ST_L_FT_INV ":"");
    result += (err->getBit(STATUS_T_MOTOR_INVALID_bit)?"ST_T_MOTOR_INV ":"");
    result += (err->getBit(STATUS_HP_MOTOR_INVALID_bit)?"ST_HP_MOTOR_INV ":"");
    result += (err->getBit(STATUS_HT_MOTOR_INVALID_bit)?"ST_HT_MOTOR_INV ":"");
    result += (err->getBit(LWR_NAN_LIM_bit)?"ST_LWR_NAN_LIM ":"");
    result += (err->getBit(COMMAND_bit)?"CMD ":"");
    result += (err->getBit(CMD_T_MOTOR_INVALID_bit)?"CMD_T_MOTOR_INV ":"");
    result += (err->getBit(CMD_HP_MOTOR_INVALID_bit)?"CMD_HP_MOTOR_INV ":"");
    result += (err->getBit(CMD_HT_MOTOR_INVALID_bit)?"CMD_HT_MOTOR_INV ":"");
    result += (err->getBit(CMD_L_ARM_INVALID_bit)?"CMD_L_ARM_INV ":"");
    result += (err->getBit(CMD_R_ARM_INVALID_bit)?"CMD_R_ARM_INV ":"");
    result += (err->getBit(CMD_R_ARM_NAN_bit)?"CMD_R_ARM_NAN ":"");
    result += (err->getBit(CMD_L_ARM_NAN_bit)?"CMD_L_ARM_NAN ":"");
    result += (err->getBit(CMD_R_ARM_LIM_bit)?"CMD_R_ARM_LIM ":"");
    result += (err->getBit(CMD_L_ARM_LIM_bit)?"CMD_L_ARM_LIM ":"");
    result += (err->getBit(CMD_T_MOTOR_T_NAN_bit)?"CMD_T_MOTOR_T_NAN ":"");

    return result;
}

