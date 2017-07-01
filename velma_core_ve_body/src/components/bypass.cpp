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

#include <controller_common/bypass.h>

#include <rtt/Component.hpp>
#include <Eigen/Dense>

using namespace RTT;

namespace velma_core_ve_body_types {

class BypassCommand: public controller_common::BypassComponent {
public:
    explicit BypassCommand(const std::string &name);

private:
    typedef Eigen::Matrix<double, 7, 1 > ArmJoints;
    typedef Eigen::Matrix<double, 4, 1 > HandDofs;
};

BypassCommand::BypassCommand(const std::string &name)
    : BypassComponent(name)
{
    addInputPort( std::make_shared< RTT::InputPort<ArmJoints > >("lArm_t_INPORT") );
    addInputPort( std::make_shared< RTT::InputPort<ArmJoints > >("rArm_t_INPORT") );
    addInputPort( std::make_shared< RTT::InputPort<int32_t > >("tact_INPORT") );
    addInputPort( std::make_shared< RTT::InputPort<HandDofs > >("rHand_q_INPORT") );
    addInputPort( std::make_shared< RTT::InputPort<HandDofs > >("lHand_q_INPORT") );
    addInputPort( std::make_shared< RTT::InputPort<HandDofs > >("rHand_dq_INPORT") );
    addInputPort( std::make_shared< RTT::InputPort<HandDofs > >("lHand_dq_INPORT") );
    addInputPort( std::make_shared< RTT::InputPort<double > >("rHand_max_p_INPORT") );
    addInputPort( std::make_shared< RTT::InputPort<double > >("lHand_max_p_INPORT") );
    addInputPort( std::make_shared< RTT::InputPort<HandDofs > >("rHand_max_i_INPORT") );
    addInputPort( std::make_shared< RTT::InputPort<HandDofs > >("lHand_max_i_INPORT") );
    addInputPort( std::make_shared< RTT::InputPort<int32_t > >("rHand_hold_INPORT") );
    addInputPort( std::make_shared< RTT::InputPort<int32_t > >("lHand_hold_INPORT") );
    addInputPort( std::make_shared< RTT::InputPort<uint8_t > >("rHandReset_INPORT") );
    addInputPort( std::make_shared< RTT::InputPort<uint8_t > >("lHandReset_INPORT") );
}

}   //namespace velma_core_ve_body_types

ORO_LIST_COMPONENT_TYPE(velma_core_ve_body_types::BypassCommand)

