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

#ifndef DOUBLE_JOINT_CONSTRAINT_H__
#define DOUBLE_JOINT_CONSTRAINT_H__

#include <cstring>

#include <vector>
#include <string>
#include <sstream>

#include "Eigen/Dense"

#include "rtt/RTT.hpp"

#include "planer_utils/activation_function.h"
#include "planer_utils/double_joint_collision_checker.h"

template<unsigned DOFS >
class DoubleJointConstraint: public RTT::TaskContext {
 public:
  explicit DoubleJointConstraint(const std::string &name);

  bool configureHook();

  bool startHook();

  void updateHook();

  std::string getDiag();

  bool inCollision(double q0, double q1) const;

 private:

  typedef Eigen::Matrix<double, DOFS, 1> Joints;
  typedef Eigen::Matrix<double, DOFS, DOFS> Inertia;
  typedef Eigen::Matrix<double, 1, DOFS> Jacobian;
  typedef Eigen::Matrix<double, DOFS, 1> JacobianT;

  boost::shared_ptr<DoubleJointCC > cc_;
  boost::shared_ptr<ActivationFunction > af_;

  RTT::InputPort<Joints> port_joint_position_;
  RTT::InputPort<Joints> port_joint_velocity_;
  RTT::InputPort<Inertia> port_mass_matrix_inv_;

  RTT::InputPort<Joints> port_nullspace_torque_command_;

  RTT::OutputPort<Joints> port_joint_torque_command_;

  Joints joint_position_;
  Joints joint_velocity_;

  Joints joint_torque_command_;
  Joints nullspace_torque_command_;

  Jacobian J;
  JacobianT JT;
  Inertia M, Minv, P;

  DoubleJointCC::Joints diag_min_v_;
  double diag_min_dist_;

  // Orocos properties / ROS parameters
  double d0_;
  unsigned joint0_idx_;
  unsigned joint1_idx_;
  std::vector<double > constraint_polygon_;
};

#ifdef EIGEN_RUNTIME_NO_MALLOC
#define RESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(false)
#define UNRESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(true)
#else
#error EIGEN_RUNTIME_NO_MALLOC must be defined
#endif

template<unsigned DOFS >
  DoubleJointConstraint<DOFS >::DoubleJointConstraint(const std::string &name)
    : RTT::TaskContext(name, PreOperational)
    , port_joint_torque_command_("JointTorqueCommand_OUTPORT", false)
    , port_joint_position_("JointPosition_INPORT")
    , port_joint_velocity_("JointVelocity_INPORT")
    , port_mass_matrix_inv_("MassMatrixInv_INPORT")
    , port_nullspace_torque_command_("NullSpaceTorqueCommand_INPORT")
    , d0_(0.0)
    , joint0_idx_(0)
    , joint1_idx_(0)
  {

    this->ports()->addPort(port_joint_position_);
    this->ports()->addPort(port_joint_velocity_);
    this->ports()->addPort(port_mass_matrix_inv_);

    this->ports()->addPort(port_joint_torque_command_);
    this->ports()->addPort(port_nullspace_torque_command_);

    this->addProperty("d0", d0_);
    this->addProperty("joint0_idx", joint0_idx_);
    this->addProperty("joint1_idx", joint1_idx_);
    this->addProperty("constraint_polygon", constraint_polygon_);

    this->addOperation("getDiag", &DoubleJointConstraint<DOFS >::getDiag, this, RTT::ClientThread);
    this->addOperation("inCollision", &DoubleJointConstraint<DOFS >::inCollision, this, RTT::ClientThread);
  }

template<unsigned DOFS >
  bool DoubleJointConstraint<DOFS >::inCollision(double q0, double q1) const {
    if (cc_) {
        DoubleJointCC::Joints q(q0, q1);
        return cc_->inCollision(q);
    }
    return false;
  }

template<unsigned DOFS >
  std::string DoubleJointConstraint<DOFS >::getDiag() {
    std::ostringstream strs;
    strs << diag_min_v_.transpose() << " " << diag_min_dist_;
    return strs.str();
  }

template<unsigned DOFS >
  bool DoubleJointConstraint<DOFS >::configureHook() {
    RTT::Logger::In in("DoubleJointConstraint::configureHook");

    if (constraint_polygon_.size() == 0 || (constraint_polygon_.size()%2) != 0) {
        RTT::log(RTT::Error) << "property \'constraint_polygon\' has wrong size: " << constraint_polygon_.size() << RTT::endlog();
        return false;
    }

    if (joint0_idx_ == joint1_idx_) {
        RTT::log(RTT::Error) << "properties \'joint0_idx\' and \'joint1_idx\' have the same value: " << joint0_idx_ << RTT::endlog();
        return false;
    }

    if (d0_ == 0.0) {
        RTT::log(RTT::Error) << "property \'d0\' is not set" << RTT::endlog();
        return false;
    }

    cc_.reset(new DoubleJointCC(d0_, constraint_polygon_));
    af_.reset(new ActivationFunction(0.2 * d0_, 4.0 / d0_));

    return true;
  }

template<unsigned DOFS >
  bool DoubleJointConstraint<DOFS >::startHook() {
    return true;
  }

template<unsigned DOFS >
  void DoubleJointConstraint<DOFS >::updateHook() {

    // read inputs
    if (port_joint_position_.read(joint_position_) != RTT::NewData) {
        RTT::Logger::In in("DoubleJointConstraint::updateHook");
        error();
        RTT::log(RTT::Error) << "could not read port \'" << port_joint_position_.getName() << "\'" << RTT::endlog();
        return;
    }

    if (!joint_position_.allFinite()) {
        RTT::Logger::In in("DoubleJointConstraint::updateHook");
        error();
        RTT::log(RTT::Error) << "joint_position_ contains NaN or inf" << RTT::endlog();
        return;
    }

    port_joint_velocity_.read(joint_velocity_);
    if (!joint_velocity_.allFinite()) {
        RTT::Logger::In in("DoubleJointConstraint::updateHook");
        error();
        RTT::log(RTT::Error) << "joint_velocity_ contains NaN or inf" << RTT::endlog();
        return;
    }

    port_nullspace_torque_command_.read(nullspace_torque_command_);
    if (!nullspace_torque_command_.allFinite()) {
        RTT::Logger::In in("DoubleJointConstraint::updateHook");
        error();
        RTT::log(RTT::Error) << "nullspace_torque_command_ contains NaN or inf" << RTT::endlog();
        return;
    }

    if (port_mass_matrix_inv_.read(Minv) != RTT::NewData) {
        RTT::Logger::In in("DoubleJointConstraint::updateHook");
        error();
        RTT::log(RTT::Error) << "could not read port \'" << port_mass_matrix_inv_.getName() << "\'" << RTT::endlog();
        return;
    }

    if (!Minv.allFinite()) {
        RTT::Logger::In in("DoubleJointConstraint::updateHook");
        error();
        RTT::log(RTT::Error) << "Minv contains NaN or inf" << RTT::endlog();
        return;
    }




    DoubleJointCC::Joints q2(joint_position_(joint0_idx_), joint_position_(joint1_idx_));

    if (cc_->inCollision(q2)) {
        RTT::Logger::In in("DoubleJointConstraint::updateHook");
        error();
        RTT::log(RTT::Error) << "collision" << RTT::endlog();
        return;
    }

    int min_idx;
    int min_type;
    double min_dist;
    DoubleJointCC::Joints min_v;

    bool found = cc_->getMinDistance(q2, min_v, min_dist, min_idx, min_type);

    diag_min_v_ = min_v;
    diag_min_dist_ = min_dist;

    P.noalias() = Inertia::Identity();
    joint_torque_command_.setZero();

    if (found) {
        min_v.normalize();

        double depth = d0_ - min_dist;
        if (depth > d0_) {
            depth = d0_;
        }
        else if (depth < 0.0) {
            depth = 0.0;
        }

        double Fmax_ = 50.0;
        double f = depth / d0_;
        double Frep = Fmax_ * f * f;
        double K = 2.0 * Fmax_ / (d0_ * d0_);

        J.setZero();

        J(0, joint0_idx_) = min_v(0);
        J(0, joint1_idx_) = min_v(1);

        JT = J.transpose();

        // calculate relative velocity between points (1 dof)
        double ddij = (J * joint_velocity_)(0,0);

        double activation_ = 1.0 - af_->func_Ndes(min_dist);

        P.noalias() -= JT * activation_ * J;

        // calculate collision mass (1 dof)
        double Mdij_inv = (J * Minv * JT)(0,0);

        double D = 0;//2.0 * 0.7 * sqrt(Mdij_inv * K);  // sqrt(K/M)
        joint_torque_command_.noalias() = JT * (Frep - D * ddij);

        RTT::log(RTT::Info) << joint_torque_command_.transpose() << RTT::endlog();
    }

    if (!joint_torque_command_.allFinite()) {
        RTT::Logger::In in("DoubleJointConstraint::updateHook");
        error();
        RTT::log(RTT::Error) << "joint_torque_command_ contains NaN or inf (#1)" << RTT::endlog();
        return;
    }

    joint_torque_command_.noalias() += P * nullspace_torque_command_;

    if (!joint_torque_command_.allFinite()) {
        RTT::Logger::In in("DoubleJointConstraint::updateHook");
        error();
        RTT::log(RTT::Error) << "joint_torque_command_ contains NaN or inf (#2)" << RTT::endlog();
        return;
    }

    // write outputs
    port_joint_torque_command_.write(joint_torque_command_);
  }

#endif  // DOUBLE_JOINT_CONSTRAINT_H__

