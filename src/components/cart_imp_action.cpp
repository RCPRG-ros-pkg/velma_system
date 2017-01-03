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

#include <string>

#include "Eigen/Dense"

#include "rtt/Component.hpp"
#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"

#include "velma_core_cs_task_cs_msgs/CommandCartImp.h"
#include "velma_core_cs_task_cs_msgs/StatusCartImp.h"

//#include "cartesian_trajectory_msgs/CartesianTrajectory.h"
#include "cartesian_trajectory_msgs/CartImpAction.h"
#include "cartesian_trajectory_msgs/CartImpGoal.h"
//#include "geometry_msgs/Wrench.h"

#include "rtt_actionlib/rtt_actionlib.h"
#include "rtt_actionlib/rtt_action_server.h"

#include "rtt_rosclock/rtt_rosclock.h"
#include "eigen_conversions/eigen_msg.h"

using cartesian_trajectory_msgs::CartesianTrajectory;
using cartesian_trajectory_msgs::CartesianTrajectoryConstPtr;

class CartImpAction: public RTT::TaskContext {
 private:
  typedef actionlib::ServerGoalHandle<cartesian_trajectory_msgs::CartImpAction> GoalHandle;
  typedef boost::shared_ptr<const cartesian_trajectory_msgs::CartImpGoal> Goal;

 public:
  explicit CartImpAction(const std::string& name);
  virtual ~CartImpAction();

  bool startHook();
  void updateHook();

    bool testTrj();

 private:
  void goalCB(GoalHandle gh);
  void cancelCB(GoalHandle gh);

//  bool checkTolerance(Eigen::Affine3d err, cartesian_trajectory_msgs::CartesianTolerance tol);
//  bool checkWrenchTolerance(geometry_msgs::Wrench msr, geometry_msgs::Wrench tol);

  // interface ports for core_cs
  velma_core_cs_task_cs_msgs::StatusCartImp status_in_;
  RTT::InputPort<velma_core_cs_task_cs_msgs::StatusCartImp > port_status_in_;

  velma_core_cs_task_cs_msgs::CommandCartImpTrjPose pose_command_out_;
  RTT::OutputPort<velma_core_cs_task_cs_msgs::CommandCartImpTrjPose > port_pose_command_out_;

  velma_core_cs_task_cs_msgs::CommandCartImpTrjPose tool_command_out_;
  RTT::OutputPort<velma_core_cs_task_cs_msgs::CommandCartImpTrjPose > port_tool_command_out_;

  velma_core_cs_task_cs_msgs::CommandCartImpTrjImp imp_command_out_;
  RTT::OutputPort<velma_core_cs_task_cs_msgs::CommandCartImpTrjImp > port_imp_command_out_;


//  RTT::OutputPort<cartesian_trajectory_msgs::CartesianTrajectoryConstPtr> port_cartesian_trajectory_command_;
  RTT::InputPort<cartesian_trajectory_msgs::CartesianTrajectory> port_cartesian_trajectory_;
  RTT::InputPort<geometry_msgs::Pose> port_cartesian_position_;
  RTT::InputPort<geometry_msgs::Pose> port_cartesian_position_command_;
  RTT::InputPort<geometry_msgs::Wrench> port_cartesian_wrench_;
  rtt_actionlib::RTTActionServer<cartesian_trajectory_msgs::CartImpAction> as_;
  GoalHandle active_goal_;

  int test_counter_;
};

CartImpAction::CartImpAction(const std::string& name) 
    : RTT::TaskContext(name)
//    , port_cartesian_trajectory_command_("CartesianTrajectoryCommand_OUTPORT", true)
//    , port_cartesian_trajectory_("trajectory_INPORT")
    , port_cartesian_position_("CartesianPosition_INPORT")
    , port_cartesian_position_command_("CartesianPositionCommand_INPORT")
    , port_cartesian_wrench_("CartesianWrench_INPORT")
    , port_status_in_("cart_pose_status_INPORT")
    , port_pose_command_out_("cart_pose_command_OUTPORT")
    , port_tool_command_out_("cart_tool_command_OUTPORT")
    , port_imp_command_out_("cart_imp_command_OUTPORT")
{


//  this->ports()->addPort(port_cartesian_trajectory_command_);
  this->ports()->addPort(port_cartesian_trajectory_);
  this->ports()->addPort(port_cartesian_position_);
  this->ports()->addPort(port_cartesian_position_command_);
  this->ports()->addPort(port_cartesian_wrench_);

  as_.addPorts(this->provides());

  this->ports()->addPort(port_status_in_);
  this->ports()->addPort(port_pose_command_out_);
  this->ports()->addPort(port_tool_command_out_);
  this->ports()->addPort(port_imp_command_out_);

  as_.registerGoalCallback(boost::bind(&CartImpAction::goalCB, this, _1));
  as_.registerCancelCallback(boost::bind(&CartImpAction::cancelCB, this, _1));
}

CartImpAction::~CartImpAction() {
}

bool CartImpAction::startHook() {
  if (as_.ready()) {
    as_.start();
  } else {
    return false;
  }
  test_counter_ = 0;

  return true;
}

void CartImpAction::updateHook() {
  CartesianTrajectory trj;
/*
// interface is realized through ROS action only
  if (port_cartesian_trajectory_.read(trj) == RTT::NewData) {
    std::cout << "New trajectory point" << std::endl;
    CartesianTrajectory* trj_ptr =  new CartesianTrajectory;
    *trj_ptr = trj;
    CartesianTrajectoryConstPtr trj_cptr = CartesianTrajectoryConstPtr(trj_ptr);

//    port_cartesian_trajectory_command_.write(trj_cptr);
  }
*/

  if (active_goal_.isValid() && (active_goal_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)) {
    cartesian_trajectory_msgs::CartImpFeedback feedback;
    Eigen::Affine3d actual, desired, error;

    ros::Time now = rtt_rosclock::host_now();

// TODO: read port_status_in_ and provide feedback
    port_status_in_.read(status_in_);

    feedback.pose_desired = status_in_.cmd;
    feedback.pose_actual = status_in_.msr;
//    port_cartesian_position_.read(feedback.actual);
//    port_cartesian_position_command_.read(feedback.desired);

    tf::poseMsgToEigen(feedback.pose_actual, actual);
    tf::poseMsgToEigen(feedback.pose_desired, desired);

    error = actual.inverse() * desired;

    tf::poseEigenToMsg(error, feedback.pose_error);

    feedback.header.stamp = now;
    active_goal_.publishFeedback(feedback);

    Goal g = active_goal_.getGoal();

    if (test_counter_ > 200) {
        active_goal_.setSucceeded();
    }
    else {
        test_counter_++;
    }

/*
// TODO: check tolerances in core_cs
    if (!checkTolerance(error, g->path_tolerance)) {
      port_cartesian_trajectory_command_.write(CartesianTrajectoryConstPtr());
      cartesian_trajectory_msgs::CartesianTrajectoryResult res;
      res.error_code = cartesian_trajectory_msgs::CartesianTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      active_goal_.setAborted(res);
    }
*/
    geometry_msgs::Wrench ft;
    ft = status_in_.force;
    //port_cartesian_wrench_.read(ft);

/*
// TODO: check tolerances in core_cs
    if (!checkWrenchTolerance(ft, g->wrench_constraint)) {
      port_cartesian_trajectory_command_.write(CartesianTrajectoryConstPtr());
      cartesian_trajectory_msgs::CartesianTrajectoryResult res;
      res.error_code = cartesian_trajectory_msgs::CartesianTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      active_goal_.setAborted(res);
    }
*/

    // TODO(konradb3): check goal constraint.
    size_t last_point = g->pose_trj.points.size() - 1;

    if ((g->pose_trj.header.stamp + g->pose_trj.points[last_point].time_from_start) < now) {
      active_goal_.setSucceeded();
    }
  }
}
/*
bool CartesianTrajectoryAction::checkTolerance(Eigen::Affine3d err, cartesian_trajectory_msgs::CartesianTolerance tol) {
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

bool CartesianTrajectoryAction::checkWrenchTolerance(geometry_msgs::Wrench msr, geometry_msgs::Wrench tol) {
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
*/
void CartImpAction::goalCB(GoalHandle gh) {
    // cancel active goal
    if (active_goal_.isValid() && (active_goal_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)) {
        active_goal_.setCanceled();
    }

    Goal g = gh.getGoal();

    if (g->pose_trj.points.size() > pose_command_out_.trj.size()) {
        cartesian_trajectory_msgs::CartImpResult res;
// TODO:
//        res.error_code = 
        active_goal_.setRejected(res);
        return;
    }

    if (g->tool_trj.points.size() > tool_command_out_.trj.size()) {
        cartesian_trajectory_msgs::CartImpResult res;
// TODO:
//        res.error_code = 
        active_goal_.setRejected(res);
        return;
    }

    if (g->imp_trj.points.size() > imp_command_out_.trj.size()) {
        cartesian_trajectory_msgs::CartImpResult res;
// TODO:
//        res.error_code = 
        active_goal_.setRejected(res);
        return;
    }

    if (g->pose_trj.points.size() == 0
            && g->pose_trj.points.size() == 0
            && g->pose_trj.points.size() == 0) {
        cartesian_trajectory_msgs::CartImpResult res;
// TODO:
//        res.error_code = 
        active_goal_.setRejected(res);
        return;
    }

    if (g->pose_trj.points.size() > 0) {
        for (int i = 0; i < g->pose_trj.points.size(); ++i) {
            pose_command_out_.trj[i] = g->pose_trj.points[i];
        }
        pose_command_out_.count = g->pose_trj.points.size();
        port_pose_command_out_.write(pose_command_out_);
    }

    if (g->tool_trj.points.size() > 0) {
        for (int i = 0; i < g->tool_trj.points.size(); ++i) {
            tool_command_out_.trj[i] = g->tool_trj.points[i];
        }
        tool_command_out_.count = g->tool_trj.points.size();
        port_tool_command_out_.write(tool_command_out_);
    }

    if (g->imp_trj.points.size() > 0) {
        for (int i = 0; i < g->imp_trj.points.size(); ++i) {
            imp_command_out_.trj[i] = g->imp_trj.points[i];
        }
        imp_command_out_.count = g->imp_trj.points.size();
        port_imp_command_out_.write(imp_command_out_);
    }

std::cout << "CartImpAction: sending trajectory" << std::endl;
    gh.setAccepted();
    active_goal_ = gh;

    test_counter_ = 0;
//  CartesianTrajectory* trj_ptr =  new CartesianTrajectory;
//  *trj_ptr = g->trajectory;
//  CartesianTrajectoryConstPtr trj_cptr = CartesianTrajectoryConstPtr(trj_ptr);
//  port_cartesian_trajectory_command_.write(trj_cptr);

}

void CartImpAction::cancelCB(GoalHandle gh) {
    if (active_goal_ == gh) {
        pose_command_out_ = velma_core_cs_task_cs_msgs::CommandCartImpTrjPose();
        port_pose_command_out_.write(pose_command_out_);

        active_goal_.setCanceled();
    }
}

ORO_LIST_COMPONENT_TYPE(CartImpAction)

