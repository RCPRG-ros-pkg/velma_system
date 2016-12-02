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

#include "velma_core_cs_ve_body_interface/command_ports.h"

#include "Eigen/Dense"

using velma_core_cs_ve_body_msgs::Command;
using velma_core_ve_body_re_body_msgs::CommandArm;
using velma_core_ve_body_re_body_msgs::CommandHand;
using velma_core_ve_body_re_body_msgs::CommandSimple;
using velma_core_ve_body_re_body_msgs::CommandMotor;

namespace velma_core_cs_ve_body_interface {

//
// ArmCommand_Ports interface
//
template <template <typename Type> class T >
ArmCommand_Ports<T >::ArmCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, CommandArm Command::*ptr) :
    PortsContainer(ptr)
{
    addPort(boost::shared_ptr<PortInterface<CommandArm > >(new Port<T, Eigen::Matrix<double,7,1>, CommandArm, CommandArm::_t_type >(tc, prefix + "_t", &CommandArm::t)));

}

//
// HandCommand_Ports interface
//
template <template <typename Type> class T >
HandCommand_Ports<T>::HandCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, CommandHand Command::*ptr) :
    PortsContainer(ptr)
{
    addPort(boost::shared_ptr<PortInterface<CommandHand > >(new Port<T, Eigen::Matrix<double,4,1>, CommandHand, CommandHand::_q_type >(tc, prefix + "_q", &CommandHand::q)));
    addPort(boost::shared_ptr<PortInterface<CommandHand > >(new Port<T, Eigen::Matrix<double,4,1>, CommandHand, CommandHand::_dq_type >(tc, prefix + "_dq", &CommandHand::dq)));
    addPort(boost::shared_ptr<PortInterface<CommandHand > >(new Port<T, Eigen::Matrix<double,4,1>, CommandHand, CommandHand::_max_i_type >(tc, prefix + "_max_i", &CommandHand::max_i)));
    addPort(boost::shared_ptr<PortInterface<CommandHand > >(new Port<T, Eigen::Matrix<double,4,1>, CommandHand, CommandHand::_max_p_type >(tc, prefix + "_max_p", &CommandHand::max_p)));
    addPort(boost::shared_ptr<PortInterface<CommandHand > >(new Port<T, bool, CommandHand, CommandHand::_hold_type >(tc, prefix + "_hold", &CommandHand::hold)));

}

//
// SimpleCommand_Ports interface
//
template <template <typename Type> class T >
SimpleCommand_Ports<T>::SimpleCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, CommandSimple Command::*ptr) :
    PortsContainer(ptr)
{
    addPort(boost::shared_ptr<PortInterface<CommandSimple > >(new Port<T, int32_t, CommandSimple, CommandSimple::_cmd_type>(tc, prefix + "_cmd", &CommandSimple::cmd)));
    addPort(boost::shared_ptr<PortInterface<CommandSimple > >(new Port<T, bool, CommandSimple, CommandSimple::_valid_type>(tc, prefix + "_valid", &CommandSimple::valid)));
}

//
// MotorCommand_Ports interface
//
template <template <typename Type> class T >
MotorCommand_Ports<T >::MotorCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, CommandMotor Command::*ptr) :
    PortsContainer(ptr)
{
    addPort(boost::shared_ptr<PortInterface<CommandMotor > >(new Port<T, double, CommandMotor, CommandMotor::_i_type >(tc, prefix + "_i", &CommandMotor::i)));
    addPort(boost::shared_ptr<PortInterface<CommandMotor > >(new Port<T, double, CommandMotor, CommandMotor::_q_type >(tc, prefix + "_q", &CommandMotor::q)));
    addPort(boost::shared_ptr<PortInterface<CommandMotor > >(new Port<T, double, CommandMotor, CommandMotor::_dq_type >(tc, prefix + "_dq", &CommandMotor::dq)));
}

//
// FTSensorCommand_Ports interface
//
template <template <typename Type> class T>
FTSensorCommand_Ports<T >::FTSensorCommand_Ports(RTT::TaskContext &tc, const std::string &prefix) {
}

//
// VelmaCommand_Ports interface
//
template <template <typename Type> class T>
VelmaCommand_Ports<T >::VelmaCommand_Ports(RTT::TaskContext &tc)
{
    addPort(boost::shared_ptr<PortInterface<Command > >(new Port<T, uint32_t, Command, Command::_test_type >(tc, "cmd_test", &Command::test)));
    addPort(boost::shared_ptr<PortInterface<Command > >(new ArmCommand_Ports<T >(tc, "cmd_rArm", &Command::rArm)), &Command::rArm_valid);
    addPort(boost::shared_ptr<PortInterface<Command > >(new ArmCommand_Ports<T >(tc, "cmd_lArm", &Command::lArm)), &Command::lArm_valid);
    addPort(boost::shared_ptr<PortInterface<Command > >(new HandCommand_Ports<T >(tc, "cmd_rHand", &Command::rHand)), &Command::rHand_valid);
    addPort(boost::shared_ptr<PortInterface<Command > >(new HandCommand_Ports<T >(tc, "cmd_lHand", &Command::lHand)), &Command::lHand_valid);
    addPort(boost::shared_ptr<PortInterface<Command > >(new SimpleCommand_Ports<T >(tc, "cmd_rTact", &Command::rTact)), &Command::rTact_valid);
    addPort(boost::shared_ptr<PortInterface<Command > >(new MotorCommand_Ports<T >(tc, "cmd_tMotor", &Command::tMotor)), &Command::tMotor_valid);
    addPort(boost::shared_ptr<PortInterface<Command > >(new MotorCommand_Ports<T >(tc, "cmd_hpMotor", &Command::hpMotor)), &Command::hpMotor_valid);
    addPort(boost::shared_ptr<PortInterface<Command > >(new MotorCommand_Ports<T >(tc, "cmd_htMotor", &Command::htMotor)), &Command::htMotor_valid);
    addPort(boost::shared_ptr<PortInterface<Command > >(new SimpleCommand_Ports<T >(tc, "cmd_safety_controller", &Command::sc)), &Command::sc_valid);

}

};  // namespace velma_core_cs_ve_body_interface

