/*
 * Copyright (c) 2010-2017, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robot Control and Pattern Recognition Group,
 *       Warsaw University of Technology nor the names of its contributors may
 *       be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * CartesianInterpolator.cpp
 *
 *  Created on: 27 lut 2014
 *      Author: konradb3, dseredyn
 */

//#include <string>

//#include "Eigen/Dense"
//#include "Eigen/Geometry"

#include "rtt/Component.hpp"
//#include "rtt/TaskContext.hpp"
//#include "rtt/Port.hpp"
//#include "rtt_rosclock/rtt_rosclock.h"

#include "velma_core_cs_task_cs_msgs/CommandCartImp.h"
//#include <velma_core_cs_task_cs_msgs/StatusCartImp.h>

//#include "cartesian_trajectory_msgs/CartesianTrajectory.h"
//#include "geometry_msgs/Pose.h"
//#include "eigen_conversions/eigen_msg.h"

/*
using namespace RTT;

class CartesianInterpolatorNew : public RTT::TaskContext {
 public:
  explicit CartesianInterpolatorNew(const std::string& name);
  virtual ~CartesianInterpolatorNew();
  virtual bool configureHook();
  virtual bool startHook();
  virtual void stopHook();
  virtual void updateHook();

 private:
  void resetTrajectory();

  geometry_msgs::Pose interpolate(
      const cartesian_trajectory_msgs::CartesianTrajectoryPoint& p0,
      const cartesian_trajectory_msgs::CartesianTrajectoryPoint& p1,
      ros::Time t);
  double interpolate(double p0, double p1, double t0, double t1, double t);

  bool checkTolerance(Eigen::Affine3d err, cartesian_trajectory_msgs::CartesianTolerance tol);
  bool checkWrenchTolerance(geometry_msgs::Wrench msr, geometry_msgs::Wrench tol);

  velma_core_cs_task_cs_msgs::CommandCartImpTrjPose trajectory_;
  RTT::InputPort<velma_core_cs_task_cs_msgs::CommandCartImpTrjPose> port_trajectory_;

  RTT::InputPort<geometry_msgs::Pose> port_cartesian_position_;

  RTT::OutputPort<geometry_msgs::Pose> port_cartesian_command_;

  int32_t generator_status_;
  RTT::OutputPort<int32_t> port_generator_status_out_;

  geometry_msgs::Pose setpoint_;
  geometry_msgs::Pose old_point_;

  size_t trajectory_idx_;

  bool activate_pose_init_property_;
  geometry_msgs::Pose init_setpoint_property_;

  bool last_point_not_set_;
  bool trajectory_active_;

  bool check_tolerances_;
};

CartesianInterpolatorNew::CartesianInterpolatorNew(const std::string& name)
    : RTT::TaskContext(name),
      trajectory_idx_(0),
      activate_pose_init_property_(false),
      last_point_not_set_(false),
      trajectory_active_(false),
      check_tolerances_(true),
      port_cartesian_position_("CartesianPosition_INPORT"),
      port_cartesian_command_("CartesianPositionCommand_OUTPORT", true),
      port_trajectory_("CartesianTrajectoryCommand_INPORT"),
      port_generator_status_out_("generator_status_OUTPORT")
{

  this->ports()->addPort(port_cartesian_position_).doc("data type: geometry_msgs::Pose");
  this->ports()->addPort(port_cartesian_command_).doc("data type: geometry_msgs::Pose");
  this->ports()->addPort(port_trajectory_).doc("data type: velma_core_cs_task_cs_msgs::CommandCartImpTrjPose");
  this->ports()->addPort(port_generator_status_out_).doc("data type: int32");

  this->addProperty("activate_pose_init", activate_pose_init_property_);
  this->addProperty("init_setpoint", init_setpoint_property_);
  this->addProperty("check_tolerances", check_tolerances_);
}

CartesianInterpolatorNew::~CartesianInterpolatorNew() {
}

bool CartesianInterpolatorNew::configureHook() {
  return true;
}

bool CartesianInterpolatorNew::startHook() {
  if (activate_pose_init_property_) {
    setpoint_ = init_setpoint_property_;
  } else {
    if (port_cartesian_position_.read(setpoint_) != RTT::NewData) {
      Logger::In in("CartesianInterpolator::startHook");
      Logger::log() << Logger::Error << "could not read data on port "
                    << port_cartesian_position_.getName() << Logger::endl;
      return false;
    }
  }

  last_point_not_set_ = false;
  trajectory_active_ = false;
  generator_status_ = velma_core_cs_task_cs_msgs::StatusCartImp::INACTIVE;

  return true;
}

void CartesianInterpolatorNew::stopHook() {
}

void CartesianInterpolatorNew::updateHook() {

  if (port_trajectory_.read(trajectory_) == RTT::NewData) {
    trajectory_idx_ = 0;
    old_point_ = setpoint_;
    last_point_not_set_ = true;
    trajectory_active_ = true;
    generator_status_ = velma_core_cs_task_cs_msgs::StatusCartImp::ACTIVE;
  }

  ros::Time now = rtt_rosclock::host_now();
  if (trajectory_active_ && (trajectory_.start < now)) {
    for (; trajectory_idx_ < trajectory_.count; trajectory_idx_++) {
      ros::Time trj_time = trajectory_.start + trajectory_.trj[trajectory_idx_].time_from_start;
      if (trj_time > now) {
        break;
      }
    }

    if (trajectory_idx_ < trajectory_.count) {
      if (trajectory_idx_ == 0) {
        cartesian_trajectory_msgs::CartesianTrajectoryPoint p0;
        p0.time_from_start.fromSec(0.0);
        p0.pose = old_point_;
        setpoint_ = interpolate(p0, trajectory_.trj[trajectory_idx_], now);
      } else {
        setpoint_ = interpolate(trajectory_.trj[trajectory_idx_ - 1],
                                trajectory_.trj[trajectory_idx_], now);
      }
    } else if (last_point_not_set_) {
      setpoint_ = trajectory_.trj[trajectory_.count - 1].pose;
      last_point_not_set_ = false;
    }
  }

  if (check_tolerances_) {
    geometry_msgs::Pose pose_msr;
    if (port_cartesian_position_.read(pose_msr) != RTT::NewData) {
      error();
      return;
    }

    Eigen::Affine3d actual, desired, error;

    tf::poseMsgToEigen(pose_msr, actual);
    tf::poseMsgToEigen(setpoint_, desired);
    error = actual.inverse() * desired;

    if (!checkTolerance(error, trajectory_.path_tolerance)) {
      resetTrajectory();
      generator_status_ = velma_core_cs_task_cs_msgs::StatusCartImp::PATH_TOLERANCE_VIOLATED;
    }
    else if (trajectory_idx_ > 0 && trajectory_idx_ == trajectory_.count) {
          ros::Time goal_time = trajectory_.start + trajectory_.trj[trajectory_.count - 1].time_from_start;
          // check goal tolerance
          bool goal_reached = checkTolerance(error, trajectory_.goal_tolerance);

          if (now > goal_time + trajectory_.goal_time_tolerance) {
              if (goal_reached) {
                  resetTrajectory();
                  generator_status_ = velma_core_cs_task_cs_msgs::StatusCartImp::SUCCESSFUL;
              }
              else {
                  resetTrajectory();
                  generator_status_ = velma_core_cs_task_cs_msgs::StatusCartImp::GOAL_TOLERANCE_VIOLATED;
              }
          }
          else if (now > goal_time - trajectory_.goal_time_tolerance) {
              if (goal_reached) {
                  resetTrajectory();
                  generator_status_ = velma_core_cs_task_cs_msgs::StatusCartImp::SUCCESSFUL;
              }
          }
    }
  }
  else {
    if (trajectory_idx_ > 0 && trajectory_idx_ == trajectory_.count && !last_point_not_set_) {
      resetTrajectory();
      generator_status_ = velma_core_cs_task_cs_msgs::StatusCartImp::SUCCESSFUL;
    }
  }

  port_generator_status_out_.write(generator_status_);

  port_cartesian_command_.write(setpoint_);
}

void CartesianInterpolatorNew::resetTrajectory() {
  trajectory_idx_ = 0;
  trajectory_ = velma_core_cs_task_cs_msgs::CommandCartImpTrjPose();
}

geometry_msgs::Pose CartesianInterpolatorNew::interpolate(
    const cartesian_trajectory_msgs::CartesianTrajectoryPoint& p0,
    const cartesian_trajectory_msgs::CartesianTrajectoryPoint& p1,
    ros::Time t) {
  geometry_msgs::Pose pose;

  ros::Time t0 = trajectory_.start + p0.time_from_start;
  ros::Time t1 = trajectory_.start + p1.time_from_start;

  pose.position.x = interpolate(p0.pose.position.x, p1.pose.position.x,
                                t0.toSec(), t1.toSec(), t.toSec());
  pose.position.y = interpolate(p0.pose.position.y, p1.pose.position.y,
                                t0.toSec(), t1.toSec(), t.toSec());
  pose.position.z = interpolate(p0.pose.position.z, p1.pose.position.z,
                                t0.toSec(), t1.toSec(), t.toSec());

  Eigen::Quaterniond q0(p0.pose.orientation.w, p0.pose.orientation.x,
                        p0.pose.orientation.y, p0.pose.orientation.z);
  Eigen::Quaterniond q1(p1.pose.orientation.w, p1.pose.orientation.x,
                        p1.pose.orientation.y, p1.pose.orientation.z);

  double a = interpolate(0.0, 1.0, t0.toSec(), t1.toSec(), t.toSec());
  Eigen::Quaterniond q = q0.slerp(a, q1);
  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();

  return pose;
}

double CartesianInterpolatorNew::interpolate(double p0, double p1, double t0,
                                          double t1, double t) {
  return (p0 + (p1 - p0) * (t - t0) / (t1 - t0));
}

bool CartesianInterpolatorNew::checkTolerance(Eigen::Affine3d err, cartesian_trajectory_msgs::CartesianTolerance tol) {
  if ((tol.position.x > 0.0) && (fabs(err.translation().x()) > tol.position.x)) {
    return false;
  }

  if ((tol.position.y > 0.0) && (fabs(err.translation().y()) > tol.position.y)) {
    return false;
  }

  if ((tol.position.z > 0.0) && (fabs(err.translation().z()) > tol.position.z)) {
    return false;
  }

  Eigen::AngleAxisd ax(err.rotation());
  Eigen::Vector3d rot = ax.axis() * ax.angle();

  if ((tol.rotation.x > 0.0) && (fabs(rot(0)) > tol.rotation.x)) {
    return false;
  }

  if ((tol.rotation.y > 0.0) && (fabs(rot(1)) > tol.rotation.y)) {
    return false;
  }

  if ((tol.rotation.z > 0.0) && (fabs(rot(2)) > tol.rotation.z)) {
    return false;
  }

  return true;
}

bool CartesianInterpolatorNew::checkWrenchTolerance(geometry_msgs::Wrench msr, geometry_msgs::Wrench tol) {
  if ((tol.force.x > 0.0) && (fabs(msr.force.x) > tol.force.x)) {
    return false;
  }

  if ((tol.force.y > 0.0) && (fabs(msr.force.y) > tol.force.y)) {
    return false;
  }

  if ((tol.force.z > 0.0) && (fabs(msr.force.z) > tol.force.z)) {
    return false;
  }

  if ((tol.torque.x > 0.0) && (fabs(msr.torque.x) > tol.torque.x)) {
    return false;
  }

  if ((tol.torque.y > 0.0) && (fabs(msr.torque.y) > tol.torque.y)) {
    return false;
  }

  if ((tol.torque.z > 0.0) && (fabs(msr.torque.z) > tol.torque.z)) {
    return false;
  }

  return true;
}

ORO_LIST_COMPONENT_TYPE(CartesianInterpolatorNew)
*/

#include <controller_common/cartesian_interpolator.h>

typedef CartesianInterpolator<velma_core_cs_task_cs_msgs::CommandCartImpTrjPose> CartesianInterpolatorVelma;

ORO_LIST_COMPONENT_TYPE(CartesianInterpolatorVelma)

