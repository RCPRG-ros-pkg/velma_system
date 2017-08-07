/*
 Copyright (c) 2014-2017, Robot Control and Pattern Recognition Group, Warsaw University of Technology
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

#include "rtt/TaskContext.hpp"
#include <rtt/Logger.hpp>
#include "rtt/Component.hpp"

#include "behavior_switch_action_msgs/BehaviorSwitchAction.h"
#include "behavior_switch_action_msgs/BehaviorSwitchGoal.h"

#include "rtt_actionlib/rtt_actionlib.h"
#include "rtt_actionlib/rtt_action_server.h"

#include "rtt_rosclock/rtt_rosclock.h"

class SafeColAction: public RTT::TaskContext {
 private:
  typedef actionlib::ServerGoalHandle<behavior_switch_action_msgs::BehaviorSwitchAction> GoalHandle;
  typedef boost::shared_ptr<const behavior_switch_action_msgs::BehaviorSwitchGoal> Goal;

 public:
  explicit SafeColAction(const std::string& name);
  virtual ~SafeColAction();

  bool startHook();
  void updateHook();

 private:
  void goalCB(GoalHandle gh);

  RTT::OutputPort<uint8_t > port_cmd_out_;

  rtt_actionlib::RTTActionServer<behavior_switch_action_msgs::BehaviorSwitchAction> as_;
  GoalHandle activeGoal_;
  bool goal_active_;
};

SafeColAction::SafeColAction(const std::string& name) 
    : RTT::TaskContext(name)
    , port_cmd_out_("cmd_OUTPORT")
    , goal_active_(false)
{
  as_.addPorts(this->provides());

  this->ports()->addPort(port_cmd_out_);

  as_.registerGoalCallback(boost::bind(&SafeColAction::goalCB, this, _1));
}

SafeColAction::~SafeColAction() {
}

bool SafeColAction::startHook() {
  if (as_.ready()) {
    as_.start();
  } else {
    return false;
  }
  return true;
}

void SafeColAction::updateHook() {
  behavior_switch_action_msgs::BehaviorSwitchResult res;

  if (goal_active_) {
      res.result = 0;
      activeGoal_.setSucceeded(res, "");
      goal_active_ = false;
  }
}

void SafeColAction::goalCB(GoalHandle gh) {
    RTT::Logger::In in(std::string("SafeColAction(") + getName() + ")::goalCB");

    // cancel active goal
    if (activeGoal_.isValid() && (activeGoal_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)) {
        activeGoal_.setCanceled();
    }

    Goal g = gh.getGoal();

    gh.setAccepted();
    activeGoal_ = gh;

    uint8_t cmd = true;
    port_cmd_out_.write(cmd);

    goal_active_ = true;
}

ORO_LIST_COMPONENT_TYPE(SafeColAction)

