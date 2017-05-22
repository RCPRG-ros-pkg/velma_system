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

#include <vector>
#include <string>

#include "rtt/RTT.hpp"
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>
#include <Eigen/Dense>

#include "velma_core_cs_ve_body_msgs/StatusSC.h"

using namespace RTT;

namespace velma_core_ve_body_types {

class BypassComponent: public RTT::TaskContext {
public:
    explicit BypassComponent(const std::string &name);

    bool configureHook();

    bool startHook();

    void stopHook();

    void updateHook();

    std::string getDiag();

private:
    typedef Eigen::Matrix<double, 7, 1 > ArmJoints;
    typedef Eigen::Matrix<double, 4, 1 > HandDofs;

    RTT::InputPort<double > port_cmd_tMotor_i_in_;
    RTT::InputPort<ArmJoints > port_cmd_lArm_t_in_;
    RTT::InputPort<ArmJoints > port_cmd_rArm_t_in_;
    RTT::InputPort<int32_t > port_cmd_tact_in_;
    RTT::InputPort<double > port_cmd_hpMotor_q_in_;
    RTT::InputPort<double > port_cmd_htMotor_q_in_;

    RTT::InputPort<HandDofs > port_cmd_rHand_q_in_;
    RTT::InputPort<HandDofs > port_cmd_lHand_q_in_;
    RTT::InputPort<HandDofs > port_cmd_rHand_dq_in_;
    RTT::InputPort<HandDofs > port_cmd_lHand_dq_in_;

    RTT::InputPort<double > port_cmd_rHand_max_p_in_;
    RTT::InputPort<double > port_cmd_lHand_max_p_in_;

    RTT::InputPort<HandDofs > port_cmd_rHand_max_i_in_;
    RTT::InputPort<HandDofs > port_cmd_lHand_max_i_in_;

    RTT::InputPort<int32_t > port_cmd_rHand_hold_in_;
    RTT::InputPort<int32_t > port_cmd_lHand_hold_in_;

    velma_core_cs_ve_body_msgs::StatusSC sc_out_;
    RTT::OutputPort<velma_core_cs_ve_body_msgs::StatusSC> port_sc_out_;

    std::vector<RTT::base::DataSourceBase::shared_ptr > ds_;
    std::vector<RTT::base::InputPortInterface* > ipi_;
    std::vector<RTT::base::OutputPortInterface* > opi_;
};

BypassComponent::BypassComponent(const std::string &name)
    : TaskContext(name, PreOperational)
    , port_cmd_tMotor_i_in_("tMotor_i_INPORT")
    , port_cmd_rArm_t_in_("rArm_t_INPORT")
    , port_cmd_lArm_t_in_("lArm_t_INPORT")
    , port_cmd_tact_in_("tact_INPORT")
    , port_cmd_hpMotor_q_in_("hpMotor_q_INPORT")
    , port_cmd_htMotor_q_in_("htMotor_q_INPORT")
    , port_cmd_rHand_q_in_("rHand_q_INPORT")
    , port_cmd_lHand_q_in_("lHand_q_INPORT")
    , port_cmd_rHand_dq_in_("rHand_dq_INPORT")
    , port_cmd_lHand_dq_in_("lHand_dq_INPORT")
    , port_cmd_rHand_max_p_in_("rHand_max_p_INPORT")
    , port_cmd_lHand_max_p_in_("lHand_max_p_INPORT")
    , port_cmd_rHand_max_i_in_("rHand_max_i_INPORT")
    , port_cmd_lHand_max_i_in_("lHand_max_i_INPORT")
    , port_cmd_rHand_hold_in_("rHand_hold_INPORT")
    , port_cmd_lHand_hold_in_("lHand_hold_INPORT")
{
    this->ports()->addPort(port_cmd_tMotor_i_in_);
    this->ports()->addPort(port_cmd_rArm_t_in_);
    this->ports()->addPort(port_cmd_lArm_t_in_);
    this->ports()->addPort(port_cmd_tact_in_);
    this->ports()->addPort(port_cmd_hpMotor_q_in_);
    this->ports()->addPort(port_cmd_htMotor_q_in_);
    this->ports()->addPort(port_cmd_rHand_q_in_);
    this->ports()->addPort(port_cmd_lHand_q_in_);
    this->ports()->addPort(port_cmd_rHand_dq_in_);
    this->ports()->addPort(port_cmd_lHand_dq_in_);
    this->ports()->addPort(port_cmd_rHand_max_p_in_);
    this->ports()->addPort(port_cmd_lHand_max_p_in_);
    this->ports()->addPort(port_cmd_rHand_max_i_in_);
    this->ports()->addPort(port_cmd_lHand_max_i_in_);
    this->ports()->addPort(port_cmd_rHand_hold_in_);
    this->ports()->addPort(port_cmd_lHand_hold_in_);

    RTT::DataFlowInterface::Ports ports = this->ports()->getPorts();
    for (int i = 0; i < ports.size(); ++i) {
        RTT::base::InputPortInterface* ipi = dynamic_cast<RTT::base::InputPortInterface* >(ports[i]);
        if (ipi) {
            RTT::base::OutputPortInterface* opi( dynamic_cast<RTT::base::OutputPortInterface* >(ipi->antiClone()) );
            std::string name = opi->getName();
            name = name.substr(0, name.length()-6) + "OUTPORT";
            opi->setName(name);
            this->ports()->addPort(*opi);
            ipi_.push_back(ipi);
            opi_.push_back(opi);
            ds_.push_back(RTT::base::DataSourceBase::shared_ptr(ipi->getDataSource()));
        }
    }

    this->ports()->addPort("sc_OUTPORT", port_sc_out_);

//    this->addOperation("getDiag", &BypassComponent::getDiag, this, RTT::ClientThread);
}

std::string BypassComponent::getDiag() {
// this method may not be RT-safe
    return "";
}

bool BypassComponent::configureHook() {
    return true;
}

bool BypassComponent::startHook() {
    return true;
}

void BypassComponent::stopHook() {
}

void BypassComponent::updateHook() {
    for (int i = 0; i < ipi_.size(); ++i) {
        if (ipi_[i]->read(ds_[i]) == RTT::NewData) {
            opi_[i]->write(ds_[i]);
        }
    }

    // no error
    sc_out_.safe_behavior = false;
    sc_out_.error = false;
    sc_out_.fault_type = 0;
    sc_out_.faulty_module_id = 0;

    port_sc_out_.write(sc_out_);
}

}   //namespace velma_core_ve_body_types

ORO_LIST_COMPONENT_TYPE(velma_core_ve_body_types::BypassComponent)

