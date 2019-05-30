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
#include <rtt/Logger.hpp>

#include "velma_core_cs_ve_body_msgs/StatusSC.h"

namespace velma_core_ve_body_types {

class SafeComponent: public RTT::TaskContext {
public:
    explicit SafeComponent(const std::string &name);

    bool startHook();

    void stopHook();

    void updateHook();

private:

    // ports
    velma_core_cs_ve_body_msgs::StatusSC sc_out_;
    RTT::OutputPort<velma_core_cs_ve_body_msgs::StatusSC > port_sc_out_;

    RTT::OutputPort<uint32_t > port_safe_iterations_out_;

    uint32_t safe_iterations_;
    bool safe_iterations_over_500_;
};

SafeComponent::SafeComponent(const std::string &name)
    : TaskContext(name)
    , safe_iterations_over_500_(false)
{
    this->ports()->addPort("sc_OUTPORT", port_sc_out_);
    this->ports()->addPort("safe_iterations_OUTPORT", port_safe_iterations_out_);

    addAttribute("safeIterationsOver500", safe_iterations_over_500_);
}

bool SafeComponent::startHook() {
    safe_iterations_ = 0;
    safe_iterations_over_500_ = false;
    return true;
}

void SafeComponent::stopHook() {
    safe_iterations_ = 0;
    safe_iterations_over_500_ = false;
}

void SafeComponent::updateHook() {
    //
    // write diagnostics information for other subsystems
    //
    sc_out_.safe_behavior = true;

    sc_out_.fault_type = 0;

    sc_out_.error = (sc_out_.fault_type != 0);

    // write diagnostic data
    port_sc_out_.write(sc_out_);

    if (safe_iterations_ < std::numeric_limits<uint32_t>::max()) {
        ++safe_iterations_;
    }

    if (safe_iterations_ > 1000) {  // it is '1000' because this component runs twice in every status/command cycle
        safe_iterations_over_500_ = true;
    }
    else {
        safe_iterations_over_500_ = false;
    }

    port_safe_iterations_out_.write(safe_iterations_);
}

}   // velma_core_ve_body_types

ORO_LIST_COMPONENT_TYPE(velma_core_ve_body_types::SafeComponent)

