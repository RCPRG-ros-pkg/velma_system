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

#include "Eigen/Dense"

#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>

class HeadSim : public RTT::TaskContext
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<double, 2, 1> Joints;
    
    // public methods
    HeadSim(std::string const& name);
    ~HeadSim();
    void updateHook();
    bool startHook();
    bool configureHook();

  protected:

    // head ports
    RTT::InputPort<Joints >      port_q_in_;
    RTT::OutputPort<Joints >     port_q_out_;
    RTT::OutputPort<Joints >     port_dq_out_;

    Joints q_in_;
    Joints q_out_;
    Joints dq_out_;

    Joints tmp_q_in_;
    Joints tmp_q_out_;
    Joints tmp_dq_out_;

    bool data_valid_;
};

using namespace RTT;

HeadSim::HeadSim(std::string const& name)
    : TaskContext(name, RTT::TaskContext::PreOperational)
    , data_valid_(false)
    , q_in_(2)
    , q_out_(2)
    , dq_out_(2)
    , port_q_in_("q_INPORT")
    , port_q_out_("q_OUTPORT")
    , port_dq_out_("dq_OUTPORT")
{
    this->ports()->addPort(port_q_in_);
    this->ports()->addPort(port_q_out_);
    this->ports()->addPort(port_dq_out_);

    q_in_.setZero();
    q_out_.setZero();
    dq_out_.setZero();
}

HeadSim::~HeadSim() {
}

void HeadSim::updateHook() {
    q_out_.setZero();
    dq_out_.setZero();

    port_q_in_.read(q_in_);
    port_dq_out_.write(dq_out_);
    port_q_out_.write(q_out_);
}

bool HeadSim::startHook() {
    return true;
}

bool HeadSim::configureHook() {
    return true;
}

ORO_LIST_COMPONENT_TYPE(HeadSim)

