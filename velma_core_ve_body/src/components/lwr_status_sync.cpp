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

#include <lwr_msgs/FriRobotState.h>
#include <lwr_msgs/FriIntfState.h>
#include <geometry_msgs/Wrench.h>

using namespace RTT;

namespace velma_core_ve_body_types {

class LwrStatusSync: public RTT::TaskContext {
public:
    explicit LwrStatusSync(const std::string &name);

    bool startHook();

    void stopHook();

    void updateHook();

private:
    typedef boost::array<double, 7 > ArmJoints;
    typedef boost::array<double, 28 > ArmMassMatrix;

    // ports
    RTT::InputPort<ArmJoints > port_q_in_;
    RTT::InputPort<ArmJoints > port_dq_in_;
    RTT::InputPort<ArmJoints > port_t_in_;
    RTT::InputPort<ArmJoints > port_gt_in_;
    RTT::InputPort<geometry_msgs::Wrench > port_w_in_;
    RTT::InputPort<ArmMassMatrix > port_mmx_in_;
    RTT::InputPort<lwr_msgs::FriIntfState > port_fri_in_;
    RTT::InputPort<lwr_msgs::FriRobotState > port_rob_in_;

    RTT::OutputPort<ArmJoints > port_q_out_;
    RTT::OutputPort<ArmJoints > port_dq_out_;
    RTT::OutputPort<ArmJoints > port_t_out_;
    RTT::OutputPort<ArmJoints > port_gt_out_;
    RTT::OutputPort<geometry_msgs::Wrench > port_w_out_;
    RTT::OutputPort<ArmMassMatrix > port_mmx_out_;
    RTT::OutputPort<lwr_msgs::FriIntfState > port_fri_out_;
    RTT::OutputPort<lwr_msgs::FriRobotState > port_rob_out_;

    ArmJoints q_;
    ArmJoints q_corrected_;
    ArmJoints dq_;
    ArmJoints t_;
    ArmJoints gt_;
    geometry_msgs::Wrench w_;
    ArmMassMatrix mmx_;
    lwr_msgs::FriIntfState fri_;
    lwr_msgs::FriRobotState rob_;

    bool valid_prev1_;
    bool valid_prev2_;
    bool valid_prev3_;
    bool valid_prev4_;

    double q0_offset_;
    double q1_offset_;
    double q2_offset_;
    double q3_offset_;
    double q4_offset_;
    double q5_offset_;
    double q6_offset_;
};

LwrStatusSync::LwrStatusSync(const std::string &name)
    : TaskContext(name)
    , q0_offset_(0.0)
    , q1_offset_(0.0)
    , q2_offset_(0.0)
    , q3_offset_(0.0)
    , q4_offset_(0.0)
    , q5_offset_(0.0)
    , q6_offset_(0.0)
{
    this->ports()->addPort("q_INPORT", port_q_in_);
    this->ports()->addPort("dq_INPORT", port_dq_in_);
    this->ports()->addPort("t_INPORT", port_t_in_);
    this->ports()->addPort("gt_INPORT", port_gt_in_);
    this->ports()->addPort("w_INPORT", port_w_in_);
    this->ports()->addPort("mmx_INPORT", port_mmx_in_);
    this->ports()->addPort("fri_INPORT", port_fri_in_);
    this->ports()->addPort("rob_INPORT", port_rob_in_);

    this->ports()->addPort("q_OUTPORT", port_q_out_);
    this->ports()->addPort("dq_OUTPORT", port_dq_out_);
    this->ports()->addPort("t_OUTPORT", port_t_out_);
    this->ports()->addPort("gt_OUTPORT", port_gt_out_);
    this->ports()->addPort("w_OUTPORT", port_w_out_);
    this->ports()->addPort("mmx_OUTPORT", port_mmx_out_);
    this->ports()->addPort("fri_OUTPORT", port_fri_out_);
    this->ports()->addPort("rob_OUTPORT", port_rob_out_);

    valid_prev1_ = false;
    valid_prev2_ = false;
    valid_prev3_ = false;
    valid_prev4_ = false;

    addProperty("q0_offset", q0_offset_);
    addProperty("q1_offset", q1_offset_);
    addProperty("q2_offset", q2_offset_);
    addProperty("q3_offset", q3_offset_);
    addProperty("q4_offset", q4_offset_);
    addProperty("q5_offset", q5_offset_);
    addProperty("q6_offset", q6_offset_);
}

bool LwrStatusSync::startHook() {
    return true;
}

void LwrStatusSync::stopHook() {
}

void LwrStatusSync::updateHook() {
    bool valid = true;
    valid &= (port_q_in_.read(q_) == RTT::NewData);
    valid &= (port_dq_in_.read(dq_) == RTT::NewData);
    valid &= (port_t_in_.read(t_) == RTT::NewData);
    valid &= (port_gt_in_.read(gt_) == RTT::NewData);
    valid &= (port_w_in_.read(w_) == RTT::NewData);
    valid &= (port_mmx_in_.read(mmx_) == RTT::NewData);
    valid &= (port_fri_in_.read(fri_) == RTT::NewData);
    valid &= (port_rob_in_.read(rob_) == RTT::NewData);

    if (valid || valid_prev1_ || valid_prev2_ || valid_prev3_ || valid_prev4_) {
        q_corrected_[0] = q_[0] + q0_offset_;
        q_corrected_[1] = q_[1] + q1_offset_;
        q_corrected_[2] = q_[2] + q2_offset_;
        q_corrected_[3] = q_[3] + q3_offset_;
        q_corrected_[4] = q_[4] + q4_offset_;
        q_corrected_[5] = q_[5] + q5_offset_;
        q_corrected_[6] = q_[6] + q6_offset_;
        port_q_out_.write(q_corrected_);
        port_dq_out_.write(dq_);
        port_t_out_.write(t_);
        port_gt_out_.write(gt_);
        port_w_out_.write(w_);
        port_mmx_out_.write(mmx_);
        port_fri_out_.write(fri_);
        port_rob_out_.write(rob_);
    }
    else {
//        Logger::log() << Logger::Info << getName() << ": lost data " << Logger::endl;
    }

//    if (!valid_prev_) {
//        Logger::log() << Logger::Info << getName() << ": almost lost data " << Logger::endl;
//    }

    valid_prev4_ = valid_prev3_;
    valid_prev3_ = valid_prev2_;
    valid_prev2_ = valid_prev1_;
    valid_prev1_ = valid;
}

}   // velma_core_ve_body_types

ORO_LIST_COMPONENT_TYPE(velma_core_ve_body_types::LwrStatusSync)

