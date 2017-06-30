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

#include "velma_core_cs_ve_body_msgs/StatusSC.h"

using namespace RTT;

static int setBit(int bitfield, int bit_num, bool value) {
    if (value) {
        // set
        bitfield |= (1 << bit_num);
    }
    else {
        // clear
        bitfield &= ~(1 << bit_num);
    }
    return bitfield;
}

static bool getBit(int bitfield, int bit_num) {
    return ((bitfield & (1 << bit_num)) != 0)?true:false;
}

namespace velma_core_ve_body_types {

class SafeComponent: public RTT::TaskContext {
public:
    explicit SafeComponent(const std::string &name);

    bool startHook();

    void stopHook();

    void updateHook();

private:

    bool isNaN(double d) const;
    bool isInLim(double d, double lo_lim, double hi_lim) const;

    // ports
    velma_core_cs_ve_body_msgs::StatusSC sc_out_;
    RTT::OutputPort<velma_core_cs_ve_body_msgs::StatusSC> port_sc_out_;

    uint32_t safe_iterations_;
    bool safe_iterations_over_500_;
};

SafeComponent::SafeComponent(const std::string &name)
    : TaskContext(name)
    , safe_iterations_over_500_(false)
{
    this->ports()->addPort("sc_OUTPORT", port_sc_out_);

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
/*
    setBit(sc_out_.fault_type, velma_core_cs_ve_body_msgs::StatusSC::FAULT_COMM_HW,
        !(rArm_valid || rArm_valid_prev_) || !(lArm_valid || lArm_valid_prev_) ||
        !torso_valid || !hp_valid ||
        !ht_valid);

    setBit(sc_out_.faulty_module_id, velma_core_cs_ve_body_msgs::StatusSC::MODULE_T_MOTOR,
        !torso_valid);

    setBit(sc_out_.faulty_module_id, velma_core_cs_ve_body_msgs::StatusSC::MODULE_HP_MOTOR,
        !hp_valid);

    setBit(sc_out_.faulty_module_id, velma_core_cs_ve_body_msgs::StatusSC::MODULE_HT_MOTOR,
        !ht_valid);
*/
    sc_out_.error = (sc_out_.fault_type != 0);

// TODO: diagnostics for:
//MODULE_R_HAND
//MODULE_L_HAND
//MODULE_R_FT
//MODULE_L_FT
//MODULE_R_TACTILE
//MODULE_L_OPTOFORCE

    // write diagnostic data
    port_sc_out_.write(sc_out_);

    if (safe_iterations_ < numeric_limits<uint32_t>::max()) {
        ++safe_iterations_;
    }

    if (safe_iterations_ > 500) {
        safe_iterations_over_500_ = true;
    }
    else {
        safe_iterations_over_500_ = false;
    }
}

}   // velma_core_ve_body_types

ORO_LIST_COMPONENT_TYPE(velma_core_ve_body_types::SafeComponent)

