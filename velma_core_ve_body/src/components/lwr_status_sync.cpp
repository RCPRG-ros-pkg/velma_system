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

#include <Eigen/Dense>

using namespace RTT;

namespace velma_core_ve_body_types {

class LwrStatusSync: public RTT::TaskContext {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit LwrStatusSync(const std::string &name);

    bool startHook();

    void stopHook();

    void updateHook();

private:
    typedef Eigen::Matrix<double, 7, 1 > ArmJoints;
    typedef Eigen::Matrix<double, 7, 7 > ArmMassMatrix;

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
    ArmJoints dq_;
    ArmJoints t_;
    ArmJoints gt_;
    geometry_msgs::Wrench w_;
    ArmMassMatrix mmx_;
    lwr_msgs::FriIntfState fri_;
    lwr_msgs::FriRobotState rob_;

    bool valid_prev_;
    bool valid_prev2_;
};

LwrStatusSync::LwrStatusSync(const std::string &name)
    : TaskContext(name)
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

    valid_prev_ = false;
    valid_prev2_ = false;
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

    if (valid || valid_prev_) {// || valid_prev2_) {
        port_q_out_.write(q_);
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

    valid_prev2_ = valid_prev_;
    valid_prev_ = valid;
}

}   // velma_core_ve_body_types

ORO_LIST_COMPONENT_TYPE(velma_core_ve_body_types::LwrStatusSync)

