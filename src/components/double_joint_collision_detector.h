// Copyright (c) 2017, Robot Control and Pattern Recognition Group,
// Institute of Control and Computation Engineering
// Warsaw University of Technology
//
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Warsaw University of Technology nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: Dawid Seredynski
//

#ifndef DOUBLE_JOINT_COLLISION_DETECTOR_H__
#define DOUBLE_JOINT_COLLISION_DETECTOR_H__

#include <cstring>

#include <vector>
#include <string>
#include <sstream>

#include "Eigen/Dense"

#include "rtt/RTT.hpp"

#include "planer_utils/double_joint_collision_checker.h"

template<unsigned DOFS >
class DoubleJointCollisionDetector: public RTT::TaskContext {
 public:
  explicit DoubleJointCollisionDetector(const std::string &name);

  bool configureHook();

  bool startHook();

  void stopHook();

  void updateHook();

 private:

  typedef Eigen::Matrix<double, DOFS, 1> Joints;

  boost::shared_ptr<DoubleJointCC > cc_;
  RTT::InputPort<Joints> port_joint_position_;
  Joints joint_position_;

  // Orocos properties / ROS parameters
  unsigned joint0_idx_;
  unsigned joint1_idx_;
  std::vector<double > constraint_polygon_;

  bool in_collision_;
};

template<unsigned DOFS >
DoubleJointCollisionDetector<DOFS >::DoubleJointCollisionDetector(const std::string &name)
    : RTT::TaskContext(name, PreOperational)
    , port_joint_position_("JointPosition_INPORT")
    , joint0_idx_(0)
    , joint1_idx_(0)
    , in_collision_(false)
{

    this->ports()->addPort(port_joint_position_);

    this->addProperty("joint0_idx", joint0_idx_);
    this->addProperty("joint1_idx", joint1_idx_);
    this->addProperty("constraint_polygon", constraint_polygon_);

    this->addAttribute("inCollision", in_collision_);
}

template<unsigned DOFS >
bool DoubleJointCollisionDetector<DOFS >::configureHook() {
    RTT::Logger::In in("DoubleJointCollisionDetector::configureHook");

    if (constraint_polygon_.size() == 0 || (constraint_polygon_.size()%2) != 0) {
        RTT::log(RTT::Error) << "property \'constraint_polygon\' has wrong size: " << constraint_polygon_.size() << RTT::endlog();
        return false;
    }

    if (joint0_idx_ == joint1_idx_) {
        RTT::log(RTT::Error) << "properties \'joint0_idx\' and \'joint1_idx\' have the same value: " << joint0_idx_ << RTT::endlog();
        return false;
    }

    if (joint0_idx_ >= DOFS) {
        RTT::log(RTT::Error) << "property \'joint0_idx\' has wrong value: " << joint0_idx_ << RTT::endlog();
        return false;
    }

    if (joint1_idx_ >= DOFS) {
        RTT::log(RTT::Error) << "property \'joint1_idx\' has wrong value: " << joint1_idx_ << RTT::endlog();
        return false;
    }

    cc_.reset(new DoubleJointCC(0.0, constraint_polygon_));

    return true;
}

template<unsigned DOFS >
bool DoubleJointCollisionDetector<DOFS >::startHook() {
    in_collision_ = false;
    return true;
}

template<unsigned DOFS >
void DoubleJointCollisionDetector<DOFS >::stopHook() {
    in_collision_ = false;
}

template<unsigned DOFS >
void DoubleJointCollisionDetector<DOFS >::updateHook() {

    // read inputs
    if (port_joint_position_.read(joint_position_) == RTT::NewData) {
        DoubleJointCC::Joints q2(joint_position_(joint0_idx_), joint_position_(joint1_idx_));
        in_collision_ = cc_->inCollision(q2);
    }
    else {
        in_collision_ = false;
    }
}

#endif  // DOUBLE_JOINT_COLLISION_DETECTOR_H__

