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

#ifndef VELMA_DATA_CONVERSIONS_EIGEN_VECTOR_H_
#define VELMA_DATA_CONVERSIONS_EIGEN_VECTOR_H_

#include <string>
#include "Eigen/Dense"
#include "common_interfaces/interface_ports.h"

template <size_t DIM>
void convert(   const Eigen::Matrix<double, DIM, 1 >& data_oro,
                boost::array<double, DIM >& data_ros) {
    for (size_t i = 0; i < DIM; ++i) {
        data_ros[i] = data_oro(i);
    }
}

template <size_t DIM>
void convert(   const boost::array<double, DIM >& data_ros,
                Eigen::Matrix<double, DIM, 1 >& data_oro) {
    for (size_t i = 0; i < DIM; ++i) {
        data_oro(i) = data_ros[i];
    }
}

/*
template <size_t DIM >
class DataConversionFromEigenVector : public InputPortConv<boost::array<double, DIM >, Eigen::Matrix<double, DIM, 1 > > {
public:
    DataConversionFromEigenVector(RTT::TaskContext *tc, const std::string &port_name)
      : InputPortConv<boost::array<double, DIM >, Eigen::Matrix<double, DIM, 1 > >(tc, port_name) {
    }

    virtual void convert(const oroT& data_oro, rosT& data_ros) const {
        for (size_t i = 0; i < DIM; ++i) {
            data_ros[i] = data_oro(i);
        }
    }
};

template <size_t DIM >
class DataConversionToEigenVector : public OutputPortConv<boost::array<double, DIM >, Eigen::Matrix<double, DIM, 1 > > {
public:
    DataConversionToEigenVector(RTT::TaskContext *tc, const std::string &port_name)
      : InputPortConv<boost::array<double, DIM >, Eigen::Matrix<double, DIM, 1 > >(tc, port_name) {
    }

    virtual void convert(const rosT& data_ros, oroT& data_oro) const {
        for (size_t i = 0; i < DIM; ++i) {
            data_oro(i) = data_ros[i];
        }
    }
};
*/

/*
template <size_t DIM >
class DataConversionFromEigenVector : public DataConversion<Eigen::Matrix<double, DIM, 1 >, boost::array<double, DIM > > {
 public:
  explicit DataConversionFromEigenVector(const std::string& name)
    : DataConversion<Eigen::Matrix<double, DIM, 1 >, boost::array<double, DIM > >(name) {
  }

  virtual void convert(const Eigen::Matrix<double, DIM, 1 >& data_in, boost::array<double, DIM >& data_out) const {
    for (size_t i = 0; i < DIM; ++i) {
      data_out[i] = data_in(i);
    }
  }
};

template <size_t DIM >
class DataConversionToEigenVector: public DataConversion<boost::array<double, DIM >, Eigen::Matrix<double, DIM, 1 > > {
 public:
  explicit DataConversionToEigenVector(const std::string& name)
    : DataConversion<boost::array<double, DIM >, Eigen::Matrix<double, DIM, 1 > >(name) {
  }

  virtual void convert(const boost::array<double, DIM >& data_in, Eigen::Matrix<double, DIM, 1 >& data_out) const {
    for (size_t i = 0; i < DIM; ++i) {
      data_out(i) = data_in[i];
    }
  }
};
*/

#endif  // VELMA_DATA_CONVERSIONS_EIGEN_VECTOR_H_

