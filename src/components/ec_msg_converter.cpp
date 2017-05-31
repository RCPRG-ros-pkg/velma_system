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

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

#include <test_ec_msgs_gen/ec_msg_converter.h>

using namespace RTT;

namespace velma_core_ve_body_types {

class EcMsgConverter: public RTT::TaskContext {
public:
    explicit EcMsgConverter(const std::string &name);

    bool startHook();

    void stopHook();

    void updateHook();

private:

    test_ec_msgs_gen::EcOutputByteArray ec_bytes_in_;
    test_ec_msgs_gen::EcOutput ec_msg_out_;
    RTT::InputPort<test_ec_msgs_gen::EcOutputByteArray> port_ec_bytes_in_;
    RTT::OutputPort<test_ec_msgs_gen::EcOutput> port_ec_msg_out_;

    test_ec_msgs_gen::EcInput ec_msg_in_;
    test_ec_msgs_gen::EcInputByteArray ec_bytes_out_;
    RTT::InputPort<test_ec_msgs_gen::EcInput> port_ec_msg_in_;
    RTT::OutputPort<test_ec_msgs_gen::EcInputByteArray> port_ec_bytes_out_;
};

EcMsgConverter::EcMsgConverter(const std::string &name)
    : TaskContext(name)
    , port_ec_bytes_in_("ec_bytes_INPORT")
    , port_ec_msg_out_("ec_msg_OUTPORT")
    , port_ec_msg_in_("ec_msg_INPORT")
    , port_ec_bytes_out_("ec_bytes_OUTPORT")
{
    this->ports()->addPort(port_ec_bytes_in_);
    this->ports()->addPort(port_ec_msg_out_);
    this->ports()->addPort(port_ec_msg_in_);
    this->ports()->addPort(port_ec_bytes_out_);
}

bool EcMsgConverter::startHook() {
    return true;
}

void EcMsgConverter::stopHook() {
}

void EcMsgConverter::updateHook() {
    if (port_ec_bytes_in_.read(ec_bytes_in_) == RTT::NewData) {
        test_ec_msgs_gen::convert(ec_bytes_in_, ec_msg_out_);
        port_ec_msg_out_.write(ec_msg_out_);
    }

    if (port_ec_msg_in_.read(ec_msg_in_) == RTT::NewData) {
        test_ec_msgs_gen::convert(ec_msg_in_, ec_bytes_out_);
        port_ec_bytes_out_.write(ec_bytes_out_);
    }
}

}   //namespace velma_core_ve_body_types

ORO_LIST_COMPONENT_TYPE(velma_core_ve_body_types::EcMsgConverter)

