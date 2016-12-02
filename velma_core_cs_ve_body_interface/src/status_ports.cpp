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

#include "velma_core_cs_ve_body_interface/status_ports.h"

#include "Eigen/Dense"

using velma_core_cs_ve_body_msgs::Status;
using velma_core_ve_body_re_body_msgs::StatusArm;
using velma_core_ve_body_re_body_msgs::StatusHand;
using velma_core_ve_body_re_body_msgs::StatusMotor;
using velma_core_ve_body_re_body_msgs::StatusFT;

namespace velma_core_cs_ve_body_interface {

//
// ArmStatus_Ports interface
//
template <template <typename Type> class T >
ArmStatus_Ports<T >::ArmStatus_Ports(RTT::TaskContext &tc, const std::string &prefix, StatusArm Status::*ptr) :
    PortsContainer(ptr)
{
    addPort(boost::shared_ptr<PortInterface<StatusArm > >(new Port<T, Eigen::Matrix<double,7,1>, StatusArm, StatusArm::_q_type >(tc, prefix + "_q", &StatusArm::q)));
    addPort(boost::shared_ptr<PortInterface<StatusArm > >(new Port<T, Eigen::Matrix<double,7,1>, StatusArm, StatusArm::_dq_type >(tc, prefix + "_dq", &StatusArm::dq)));
    addPort(boost::shared_ptr<PortInterface<StatusArm > >(new Port<T, Eigen::Matrix<double,7,1>, StatusArm, StatusArm::_t_type >(tc, prefix + "_t", &StatusArm::t)));
    addPort(boost::shared_ptr<PortInterface<StatusArm > >(new Port<T, Eigen::Matrix<double,7,1>, StatusArm, StatusArm::_gt_type >(tc, prefix + "_gt", &StatusArm::gt)));
    addPort(boost::shared_ptr<PortInterface<StatusArm > >(new Port<T, geometry_msgs::Wrench, StatusArm, StatusArm::_w_type >(tc, prefix + "_w", &StatusArm::w)));
    addPort(boost::shared_ptr<PortInterface<StatusArm > >(new Port<T, Eigen::Matrix<double,7,7>, StatusArm, StatusArm::_mmx_type >(tc, prefix + "_mmx", &StatusArm::mmx)));
}

//
// HandStatus_Ports interface
//
template <template <typename Type> class T >
HandStatus_Ports<T >::HandStatus_Ports(RTT::TaskContext &tc, const std::string &prefix, StatusHand Status::*ptr) :
    PortsContainer(ptr)
{
    addPort(boost::shared_ptr<PortInterface<StatusHand > >(new Port<T, Eigen::Matrix<double,8,1>, StatusHand, StatusHand::_q_type >(tc, prefix + "_q", &StatusHand::q)));
    addPort(boost::shared_ptr<PortInterface<StatusHand > >(new Port<T, uint32_t, StatusHand, StatusHand::_s_type >(tc, prefix + "_s", &StatusHand::s)));

}

//
// MotorStatus_Ports interface
//
template <template <typename Type> class T >
MotorStatus_Ports<T >::MotorStatus_Ports(RTT::TaskContext &tc, const std::string &prefix, StatusMotor Status::*ptr) :
    PortsContainer(ptr)
{
    addPort(boost::shared_ptr<PortInterface<StatusMotor > >(new Port<T, double, StatusMotor, StatusMotor::_q_type >(tc, prefix + "_q", &StatusMotor::q)));
    addPort(boost::shared_ptr<PortInterface<StatusMotor > >(new Port<T, double, StatusMotor, StatusMotor::_dq_type >(tc, prefix + "_dq", &StatusMotor::dq)));
}

//
// FTSensorStatus_Ports interface
//
template <template <typename Type> class T >
FTSensorStatus_Ports<T >::FTSensorStatus_Ports(RTT::TaskContext &tc, const std::string &prefix, StatusFT Status::*ptr) :
    PortsContainer(ptr)
{
    addPort(boost::shared_ptr<PortInterface<StatusFT > >(new Port<T, geometry_msgs::Wrench, StatusFT, StatusFT::_rw_type >(tc, prefix + "_rw", &StatusFT::rw)));
    addPort(boost::shared_ptr<PortInterface<StatusFT > >(new Port<T, geometry_msgs::Wrench, StatusFT, StatusFT::_ffw_type >(tc, prefix + "_ffw", &StatusFT::ffw)));
    addPort(boost::shared_ptr<PortInterface<StatusFT > >(new Port<T, geometry_msgs::Wrench, StatusFT, StatusFT::_sfw_type >(tc, prefix + "_sfw", &StatusFT::sfw)));

}

//
// VelmaStatus_Ports interface
//
template <template <typename Type> class T>
VelmaStatus_Ports<T >::VelmaStatus_Ports(RTT::TaskContext &tc)
{
    addPort(boost::shared_ptr<PortInterface<Status > >(new Port<T, uint32_t, Status, Status::_test_type >(tc, "status_test", &Status::test)));
    addPort(boost::shared_ptr<PortInterface<Status > >( new ArmStatus_Ports<T >(tc, "status_rArm", &Status::rArm) ), &Status::rArm_valid);
    addPort(boost::shared_ptr<PortInterface<Status > >( new ArmStatus_Ports<T >(tc, "status_lArm", &Status::lArm) ), &Status::lArm_valid);
    addPort(boost::shared_ptr<PortInterface<Status > >( new HandStatus_Ports<T > (tc, "status_rHand", &Status::rHand) ), &Status::rHand_valid);
    addPort(boost::shared_ptr<PortInterface<Status > >( new HandStatus_Ports<T > (tc, "status_lHand", &Status::lHand) ), &Status::lHand_valid);
    addPort(boost::shared_ptr<PortInterface<Status > >( new FTSensorStatus_Ports<T > (tc, "status_rFt", &Status::rFt) ), &Status::rFt_valid);
    addPort(boost::shared_ptr<PortInterface<Status > >( new FTSensorStatus_Ports<T > (tc, "status_lFt", &Status::lFt) ), &Status::lFt_valid);
    addPort(boost::shared_ptr<PortInterface<Status > >( new MotorStatus_Ports<T > (tc, "status_tMotor", &Status::tMotor) ), &Status::tMotor_valid);
    addPort(boost::shared_ptr<PortInterface<Status > >( new MotorStatus_Ports<T > (tc, "status_hpMotor", &Status::hpMotor) ), &Status::hpMotor_valid);
    addPort(boost::shared_ptr<PortInterface<Status > >( new MotorStatus_Ports<T > (tc, "status_htMotor", &Status::htMotor) ), &Status::htMotor_valid);
    addPort(boost::shared_ptr<PortInterface<Status > >(new Port<T, barrett_hand_controller_msgs::BHPressureState, Status, Status::_rHand_p_type> (tc, "status_rHand_p", &Status::rHand_p)), &Status::rHand_p_valid);
    addPort(boost::shared_ptr<PortInterface<Status > >(new Port<T, Status::_lHand_f_type, Status, Status::_lHand_f_type> (tc, "status_lHand_f", &Status::lHand_f)), &Status::lHand_f_valid);
    addPort(boost::shared_ptr<PortInterface<Status > >(new Port<T, Status::_sc_type, Status, Status::_sc_type> (tc, "status_sc", &Status::sc)), &Status::sc_valid);
}

};  // namespace velma_core_cs_ve_body_interface

