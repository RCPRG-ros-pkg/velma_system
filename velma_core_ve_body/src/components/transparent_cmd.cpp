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

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>

#include "velma_core_cs_ve_body_msgs/StatusSC.h"

using namespace RTT;

namespace velma_core_ve_body_types {

class TransparentCmdComponent: public RTT::TaskContext {
public:
    explicit TransparentCmdComponent(const std::string &name);

    bool startHook();

    void updateHook();

private:

    // ports
    velma_core_cs_ve_body_msgs::StatusSC sc_out_;
    RTT::OutputPort<velma_core_cs_ve_body_msgs::StatusSC> port_sc_out_;
};

TransparentCmdComponent::TransparentCmdComponent(const std::string &name)
    : TaskContext(name)
{
    this->ports()->addPort("sc_OUTPORT", port_sc_out_);
}

bool TransparentCmdComponent::startHook() {
    return true;
}

void TransparentCmdComponent::updateHook() {
    // no error
    sc_out_.safe_behavior = false;
    sc_out_.error = false;
    sc_out_.fault_type = 0;
    sc_out_.faulty_module_id = 0;

    port_sc_out_.write(sc_out_);
}

}   // velma_core_ve_body_types

ORO_LIST_COMPONENT_TYPE(velma_core_ve_body_types::TransparentCmdComponent)

