// Copyright 2014 WUT
/*
 * CartesianInterpolator.cpp
 *
 *  Created on: 27 lut 2014
 *      Author: konradb3
 */

#include <string>

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "rtt/Component.hpp"
#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"
#include "rtt_rosclock/rtt_rosclock.h"

#include "velma_core_cs_task_cs_msgs/CommandCartImp.h"
#include "cartesian_trajectory_msgs/CartesianTrajectory.h"
#include "geometry_msgs/Pose.h"

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
  geometry_msgs::Pose interpolate(
      const cartesian_trajectory_msgs::CartesianTrajectoryPoint& p0,
      const cartesian_trajectory_msgs::CartesianTrajectoryPoint& p1,
      ros::Time t);
  double interpolate(double p0, double p1, double t0, double t1, double t);
  RTT::InputPort<velma_core_cs_task_cs_msgs::CommandCartImpTrjPose> port_trajectory_;
  RTT::InputPort<geometry_msgs::Pose> port_cartesian_position_;

  RTT::OutputPort<geometry_msgs::Pose> port_cartesian_command_;
  RTT::OutputPort<bool> port_generator_active_;

  velma_core_cs_task_cs_msgs::CommandCartImpTrjPose trajectory_;
  geometry_msgs::Pose setpoint_;
  geometry_msgs::Pose old_point_;

  size_t trajectory_ptr_;

  bool activate_pose_init_property_;
  geometry_msgs::Pose init_setpoint_property_;

  bool last_point_not_set_;
  bool trajectory_active_;
};

CartesianInterpolatorNew::CartesianInterpolatorNew(const std::string& name)
    : RTT::TaskContext(name),
      trajectory_ptr_(0),
      activate_pose_init_property_(false),
      last_point_not_set_(false),
      trajectory_active_(false),
      port_cartesian_position_("CartesianPosition_INPORT"),
      port_cartesian_command_("CartesianPositionCommand_OUTPORT", true),
      port_trajectory_("CartesianTrajectoryCommand_INPORT"),
      port_generator_active_("GeneratorActiveOut_OUTPORT", true)
{

  this->ports()->addPort(port_cartesian_position_).doc("data type: geometry_msgs::Pose");
  this->ports()->addPort(port_cartesian_command_).doc("data type: geometry_msgs::Pose");
  this->ports()->addPort(port_trajectory_).doc("data type: velma_core_cs_task_cs_msgs::CommandCartImpTrjPose");
  this->ports()->addPort(port_generator_active_).doc("data type: bool");

  this->addProperty("activate_pose_init", activate_pose_init_property_);
  this->addProperty("init_setpoint", init_setpoint_property_);
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

  port_generator_active_.write(true);
  last_point_not_set_ = false;
  trajectory_active_ = false;
  return true;
}

void CartesianInterpolatorNew::stopHook() {
  port_generator_active_.write(false);
}

void CartesianInterpolatorNew::updateHook() {
  port_generator_active_.write(true);
  if (port_trajectory_.read(trajectory_) == RTT::NewData) {
    trajectory_ptr_ = 0;
    old_point_ = setpoint_;
    last_point_not_set_ = true;
    trajectory_active_ = true;
  }

  ros::Time now = rtt_rosclock::host_now();
  if (trajectory_active_ && (trajectory_.start < now)) {
    for (; trajectory_ptr_ < trajectory_.count; trajectory_ptr_++) {
      ros::Time trj_time = trajectory_.start + trajectory_.trj[trajectory_ptr_].time_from_start;
      if (trj_time > now) {
        break;
      }
    }

    if (trajectory_ptr_ < trajectory_.count) {
      if (trajectory_ptr_ == 0) {
        cartesian_trajectory_msgs::CartesianTrajectoryPoint p0;
        p0.time_from_start.fromSec(0.0);
        p0.pose = old_point_;
        setpoint_ = interpolate(p0, trajectory_.trj[trajectory_ptr_], now);
      } else {
        setpoint_ = interpolate(trajectory_.trj[trajectory_ptr_ - 1],
                                trajectory_.trj[trajectory_ptr_], now);
      }
    } else if (last_point_not_set_) {
      setpoint_ = trajectory_.trj[trajectory_.count - 1].pose;
      last_point_not_set_ = false;
    }
  }
  port_cartesian_command_.write(setpoint_);
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

ORO_LIST_COMPONENT_TYPE(CartesianInterpolatorNew)

