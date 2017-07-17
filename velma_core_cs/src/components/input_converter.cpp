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

#include <common_behavior/abstract_port_converter.h>

#include <rtt/Component.hpp>
#include <Eigen/Dense>

using namespace RTT;

namespace velma_core_cs_types {

class InputConverter: public common_behavior::MultiConverterComponent {
public:
    explicit InputConverter(const std::string &name);

private:
    typedef boost::array<double, 7 > ArmJoints;
};

InputConverter::InputConverter(const std::string &name)
    : common_behavior::MultiConverterComponent(name)
{
    addConverter<boost::array<double, 7 >, Eigen::Matrix<double, 7, 1 > >( "rArm_q" );
    addConverter<boost::array<double, 7 >, Eigen::Matrix<double, 7, 1 > >( "lArm_q" );
    addConverter<boost::array<double, 7 >, Eigen::Matrix<double, 7, 1 > >( "rArm_dq" );
    addConverter<boost::array<double, 7 >, Eigen::Matrix<double, 7, 1 > >( "lArm_dq" );

    addConverter<boost::array<double, 28 >, Eigen::Matrix<double, 7, 7 > >( "rArm_mmx" );
    addConverter<boost::array<double, 28 >, Eigen::Matrix<double, 7, 7 > >( "lArm_mmx" );

    addConverter<boost::array<double, 8 >, Eigen::Matrix<double, 8, 1 > >( "rHand_q" );
    addConverter<boost::array<double, 8 >, Eigen::Matrix<double, 8, 1 > >( "lHand_q" );

    addConverter<double, Eigen::Matrix<double, 1, 1 > >( "tMotor_q" );
    addConverter<double, Eigen::Matrix<double, 1, 1 > >( "tMotor_dq" );

    addConverter<double, Eigen::Matrix<double, 1, 1 > >( "hpMotor_q" );
    addConverter<double, Eigen::Matrix<double, 1, 1 > >( "hpMotor_dq" );

    addConverter<double, Eigen::Matrix<double, 1, 1 > >( "htMotor_q" );
    addConverter<double, Eigen::Matrix<double, 1, 1 > >( "htMotor_dq" );
}

}   //namespace velma_core_cs_types

ORO_LIST_COMPONENT_TYPE(velma_core_cs_types::InputConverter)

