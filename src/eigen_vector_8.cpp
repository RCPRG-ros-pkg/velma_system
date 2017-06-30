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

#include "Eigen/Dense"
#include "common_behavior/abstract_port_converter.h"

class PortConverterEigen8ToArray : public common_behavior::Converter<Eigen::Matrix<double, 8, 1>, boost::array<double, 8 > > {
public:

    virtual void convert(const Eigen::Matrix<double, 8, 1> &from, boost::array<double, 8 > &to) const {
        for (int i = 0; i < 8; ++i) {
            to[i] = from(i);
        }
    }
};

class PortConverterArrayToEigen8 : public common_behavior::Converter<boost::array<double, 8 >, Eigen::Matrix<double, 8, 1> > {
public:

    virtual void convert(const boost::array<double, 8 > &from, Eigen::Matrix<double, 8, 1> &to) const {
        for (int i = 0; i < 8; ++i) {
            to(i) = from[i];
        }
    }
};

REGISTER_PORT_CONVERTER(PortConverterEigen8ToArray);
REGISTER_PORT_CONVERTER(PortConverterArrayToEigen8);

