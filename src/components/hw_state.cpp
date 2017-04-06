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

#include <sstream>

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>
#include <rtt/base/PortInterface.hpp>
#include <kuka_lwr_fri/friComm.h>

#include "../common_predicates.h"

using namespace RTT;

namespace velma_core_ve_body_types {

class HwState: public RTT::TaskContext {
public:
    explicit HwState(const std::string &name);

    bool startHook();

    void stopHook();

    void updateHook();

    std::string getDiag();

    bool rLwrOk() const;
    bool lLwrOk() const;
    bool rLwrCmdState() const;
    bool lLwrCmdState() const;

private:

//    bool isLwrOk(const tFriRobotState& friRobot, const tFriIntfState& friIntf) const;
//    bool isLwrInCmdState(const tFriIntfState& friIntf) const;

    // ports
    tFriIntfState       rArm_fri_state_;
    RTT::InputPort<tFriIntfState > port_rArm_fri_state_in_;

    tFriRobotState      rArm_robot_state_;
    RTT::InputPort<tFriRobotState > port_rArm_robot_state_in_;

    tFriIntfState       lArm_fri_state_;
    RTT::InputPort<tFriIntfState > port_lArm_fri_state_in_;

    tFriRobotState      lArm_robot_state_;
    RTT::InputPort<tFriRobotState > port_lArm_robot_state_in_;


    bool r_lwr_ok_;
    bool l_lwr_ok_;
    bool r_lwr_cmd_;
    bool l_lwr_cmd_;
};
/*
bool HwState::isLwrOk(const tFriRobotState& friRobot, const tFriIntfState& friIntf) const {
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

bool HwState::isLwrInCmdState(const tFriIntfState& friIntf) const {
    return friIntf.state == FRI_STATE_CMD;
}
*/
bool HwState::rLwrOk() const {
    return r_lwr_ok_;
}

bool HwState::lLwrOk() const {
    return l_lwr_ok_;
}

bool HwState::rLwrCmdState() const {
    return r_lwr_cmd_;
}

bool HwState::lLwrCmdState() const {
    return l_lwr_cmd_;
}

HwState::HwState(const std::string &name)
    : TaskContext(name)
{
    this->ports()->addPort("rArm_fri_state_INPORT", port_rArm_fri_state_in_);
    this->ports()->addPort("rArm_robot_state_INPORT", port_rArm_robot_state_in_);
    this->ports()->addPort("lArm_fri_state_INPORT", port_lArm_fri_state_in_);
    this->ports()->addPort("lArm_robot_state_INPORT", port_lArm_robot_state_in_);

    this->addOperation("rLwrOk", &HwState::rLwrOk, this, RTT::ClientThread);
    this->addOperation("lLwrOk", &HwState::lLwrOk, this, RTT::ClientThread);
    this->addOperation("rLwrCmdState", &HwState::rLwrCmdState, this, RTT::ClientThread);
    this->addOperation("lLwrCmdState", &HwState::lLwrCmdState, this, RTT::ClientThread);
}

bool HwState::startHook() {
    r_lwr_ok_ = false;
    l_lwr_ok_ = false;
    r_lwr_cmd_ = false;
    l_lwr_cmd_ = false;
    return true;
}

void HwState::stopHook() {
    r_lwr_ok_ = false;
    l_lwr_ok_ = false;
    r_lwr_cmd_ = false;
    l_lwr_cmd_ = false;
}

void HwState::updateHook() {

    bool rFri_read = (port_rArm_fri_state_in_.read(rArm_fri_state_) == RTT::NewData);
    bool lFri_read = (port_lArm_fri_state_in_.read(lArm_fri_state_) == RTT::NewData);
    bool rRob_read = (port_rArm_robot_state_in_.read(rArm_robot_state_) == RTT::NewData);
    bool lRob_read = (port_lArm_robot_state_in_.read(lArm_robot_state_) == RTT::NewData);

    r_lwr_ok_ = (rFri_read && rRob_read && isLwrOk(rArm_robot_state_, rArm_fri_state_));
    l_lwr_ok_ = (lFri_read && lRob_read && isLwrOk(lArm_robot_state_, lArm_fri_state_));

    r_lwr_cmd_ = (rFri_read && rRob_read && isLwrInCmdState(rArm_fri_state_));
    l_lwr_cmd_ = (lFri_read && lRob_read && isLwrInCmdState(lArm_fri_state_));
}

}   // velma_core_ve_body_types

ORO_LIST_COMPONENT_TYPE(velma_core_ve_body_types::HwState)

