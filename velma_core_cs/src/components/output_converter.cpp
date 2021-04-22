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

#include <subsystem_common/abstract_port_converter.h>

#include <rtt/Component.hpp>
#include <Eigen/Dense>

using namespace RTT;

namespace velma_core_cs_types {

class OutputConverter: public subsystem_common::MultiConverterComponent {
public:
    explicit OutputConverter(const std::string &name);

private:
    typedef boost::array<double, 7 > ArmJoints;
};

OutputConverter::OutputConverter(const std::string &name)
    : subsystem_common::MultiConverterComponent(name)
{
    addConverter<Eigen::Matrix<double, 2, 1 >, boost::array<double, 2 > >( "head_q_desired" );
    addConverter<Eigen::Matrix<double, 2, 1 >, boost::array<double, 2 > >( "head_dq_desired" );
    addConverter<Eigen::Matrix<double, 15, 1 >, boost::array<double, 15 > >( "jnt_q_desired" );
    addConverter<Eigen::Matrix<double, 33, 1 >, boost::array<double, 33 > >( "q" );
    addConverter<Eigen::Matrix<double, 33, 1 >, boost::array<double, 33 > >( "dq" );
    addConverter<Eigen::Matrix<double, 7, 1 >, boost::array<double, 7 > >( "rArm_t" );
    addConverter<Eigen::Matrix<double, 7, 1 >, boost::array<double, 7 > >( "lArm_t" );
    addConverter<Eigen::Matrix<double, 1, 1 >, double >( "tMotor_t" );
}

}   //namespace velma_core_cs_types

ORO_LIST_COMPONENT_TYPE(velma_core_cs_types::OutputConverter)

