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

#include <sstream>

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

#include <controller_common/elmo_servo_state.h>
#include <math.h>

using namespace RTT;

using namespace controller_common::elmo_servo;

namespace velma_core_ve_body_types {

class ElmoDriver: public RTT::TaskContext {
public:
    explicit ElmoDriver(const std::string &name);

    bool configureHook();

    bool startHook();

    void stopHook();

    void updateHook();

private:
    enum ControlMode {
        PROFILE_POSITION = 1,
        PROFILE_VELOCITY = 2,
        PROFILE_CURRENT = 3,
        HOMING = 6,
        CYCLIC_CURRENT = 10,
        CYCLIC_VELOCITY = 9,
        CYCLIC_POSITION = 8
    };

    bool enable();
    void disable();
    bool beginHoming();
    bool forceHomingDone();
    bool resetFault();

    int control_mode_;

    bool isNaN(double d) const;
    bool isInLim(double d, double lo_lim, double hi_lim) const;

    // ROS parameters
    std::string control_mode_str_;

    // i/o ports
    RTT::OutputPort<int16_t > port_i_out_;
    RTT::OutputPort<int32_t > port_q_out_;
    RTT::OutputPort<int32_t > port_dq_out_;
    RTT::OutputPort<uint16_t > port_controlWord_out_;
    RTT::OutputPort<int8_t > port_modeOfOperation_out_;

    RTT::InputPort<int32_t > port_q_in_;
    RTT::InputPort<uint16_t > port_statusWord_in_;

    // status ports
    RTT::OutputPort<uint8_t > port_homing_required_out_;
    RTT::OutputPort<uint8_t > port_homing_in_progress_out_;
    RTT::OutputPort<uint8_t > port_enabled_out_;

    // command ports
    RTT::InputPort<int16_t > port_desired_i_in_;
    RTT::InputPort<int32_t > port_desired_q_in_;
    RTT::InputPort<int32_t > port_desired_dq_in_;
    RTT::InputPort<uint8_t > port_homing_start_in_;
    RTT::InputPort<uint8_t > port_enable_in_;
    RTT::InputPort<uint8_t > port_disable_in_;

    bool enable_;
    bool homing_;
    bool reset_fault_;
    bool homing_done_;
    bool atribute_homing_required_;
    bool atribute_homing_in_progress_;

    ServoState servo_state_;
};

ElmoDriver::ElmoDriver(const std::string &name)
    : TaskContext(name, PreOperational)
    , control_mode_(HOMING)
    , enable_(false)
    , homing_(false)
    , reset_fault_(false)
    , homing_done_(true)
    , atribute_homing_required_(false)
    , atribute_homing_in_progress_(false)
    , servo_state_(ServoState::INVALID)
{
    this->ports()->addPort("controlWord_OUTPORT", port_controlWord_out_);
    this->ports()->addPort("modeOfOperation_OUTPORT", port_modeOfOperation_out_);
    this->ports()->addPort("statusWord_INPORT", port_statusWord_in_);

    this->ports()->addPort("homing_required_OUTPORT", port_homing_required_out_);
    this->ports()->addPort("homing_in_progress_OUTPORT", port_homing_in_progress_out_);
    this->ports()->addPort("homing_start_INPORT", port_homing_start_in_);

    this->ports()->addPort("enabled_OUTPORT", port_enabled_out_);
    this->ports()->addPort("enable_INPORT", port_enable_in_);
    this->ports()->addPort("disable_INPORT", port_disable_in_);

    addProperty("control_mode", control_mode_str_);
    addProperty("homing_done", homing_done_);

    addAttribute("homing_required", atribute_homing_required_);
    addAttribute("homing_in_progress", atribute_homing_in_progress_);

    this->provides()->addOperation("enable", &ElmoDriver::enable, this,
                                   RTT::OwnThread);
    this->provides()->addOperation("disable", &ElmoDriver::disable, this,
                                   RTT::OwnThread);
    this->provides()->addOperation("resetFault", &ElmoDriver::resetFault, this,
                                   RTT::OwnThread);
}

bool ElmoDriver::enable() {
  if (servo_state_ == ServoState::SWITCH_ON) {
    enable_ = true;
    return true;
  } else {
    return false;
  }
}

void ElmoDriver::disable() {
  if (enable_) {
    enable_ = false;
  }
}

bool ElmoDriver::beginHoming() {
  if (homing_done_ == false) {
    if (servo_state_ == ServoState::OPERATION_ENABLED) {
      homing_ = true;
    }
  } else {
    Logger::log() << Logger::Error << "Drive not configured for homing" << Logger::endl;
  }
  return homing_;
}

bool ElmoDriver::forceHomingDone() {
  if (servo_state_ == ServoState::OPERATION_ENABLED) {
    homing_done_ = true;
  }
  return homing_done_;
}

bool ElmoDriver::resetFault() {
  if (servo_state_ == ServoState::FAULT) {
    reset_fault_ = true;
    return true;
  } else {
    return false;
  }
}

bool ElmoDriver::configureHook() {
    Logger::In in( std::string("ElmoDriver::configureHook ") + getName());

    if (control_mode_str_ == "position") {
        this->ports()->addPort("q_OUTPORT", port_q_out_);
        this->ports()->addPort("desired_q_INPORT", port_desired_q_in_);
        this->ports()->addPort("q_INPORT", port_q_in_);
        control_mode_ = CYCLIC_POSITION;
        Logger::log() << Logger::Info << "Control mode is set to '" << control_mode_str_ << "'" << Logger::endl;
    }
    else if (control_mode_str_ == "velocity") {
        this->ports()->addPort("dq_OUTPORT", port_dq_out_);
        this->ports()->addPort("desired_dq_INPORT", port_desired_dq_in_);
        control_mode_ = CYCLIC_VELOCITY;
        Logger::log() << Logger::Info << "Control mode is set to '" << control_mode_str_ << "'" << Logger::endl;
    }
    else if (control_mode_str_ == "torque") {
        this->ports()->addPort("i_OUTPORT", port_i_out_);
        this->ports()->addPort("desired_i_INPORT", port_desired_i_in_);

        control_mode_ = CYCLIC_CURRENT;
        Logger::log() << Logger::Info << "Control mode is set to '" << control_mode_str_ << "'" << Logger::endl;
    }
    else {
        Logger::log() << Logger::Error <<
            "parameter control_mode is not set or is set to illegal value: " <<
            control_mode_str_ << Logger::endl;
        return false;
    }

    if (!homing_done_) {
        this->provides()->addOperation("beginHoming", &ElmoDriver::beginHoming, this,
                                       RTT::OwnThread);
        this->provides()->addOperation("forceHomingDone",
                                       &ElmoDriver::forceHomingDone, this,
                                       RTT::OwnThread);

        atribute_homing_required_ = true;
        atribute_homing_in_progress_ = false;
    }

    return true;
}

bool ElmoDriver::startHook() {
    return true;
}

void ElmoDriver::stopHook() {
}

void ElmoDriver::updateHook() {
    uint16_t statusWord_in;
    int32_t q_in;
    int32_t dq_in;

    bool enable_prev = enable_;

    if (port_statusWord_in_.read( statusWord_in ) != RTT::NewData) {
//        Logger::log() << Logger::Error << getName() << " could not read statusWord" << Logger::endl;
        // TODO: verify this
        return;
    }
    if (control_mode_ == CYCLIC_POSITION && port_q_in_.read(q_in) != RTT::NewData) {
//        Logger::log() << Logger::Error << getName() << " could not read position" << Logger::endl;
        // TODO: verify this
        return;
    }
//    if (port_dq_in_.read(dq_in) != RTT::NewData) {
//        // TODO: verify this
//        return;
//    }

    ServoState servo_state_prev = servo_state_;
    servo_state_ = getServoState(statusWord_in);

    if (servo_state_prev != servo_state_) {
        Logger::log() << Logger::Info << getName() << " state: " << getServoStateStr(servo_state_) << Logger::endl;
    }

    uint8_t disable_in = false;
    uint8_t enable_in = false;
    if (port_disable_in_.read(disable_in) == RTT::NewData && disable_in) {
        enable_ = false;
    }
    else if (port_enable_in_.read(enable_in) == RTT::NewData && enable_in && servo_state_ == ServoState::SWITCH_ON) {
      Logger::log() << Logger::Info << getName() << " received enable command" << Logger::endl;
      enable_ = true;
    }

    uint16_t controlWord_out = 0;
    switch (servo_state_) {
        case ServoState::NOT_READY_TO_SWITCH_ON:
          enable_ = false;
          controlWord_out = 0;
          break;
        case ServoState::SWITCH_ON_DISABLED:
          enable_ = false;
          controlWord_out = 0x06;
          break;
        case ServoState::READY_TO_SWITCH_ON:
          enable_ = false;
          controlWord_out = 0x07;
          break;
        case ServoState::SWITCH_ON:
          if (enable_) {
            controlWord_out = 0x0f;
          } else {
            controlWord_out = 0x07;
          }
          break;
        case ServoState::OPERATION_ENABLED:
          if (enable_) {
            controlWord_out = 0x0f;
          } else {
            controlWord_out = 0x07;
          }
          break;
        case ServoState::QUICK_STOP_ACTIVE:
          enable_ = false;
          controlWord_out = 0x0f;
          break;
        case ServoState::FAULT_REACTION_ACTIVE:
          enable_ = false;
          break;
        case ServoState::FAULT:
          enable_ = false;
          if (reset_fault_) {
            controlWord_out = 0x80;
            reset_fault_ = false;
          } else {
            controlWord_out = 0;
          }
          break;
        default:
          break;
    }

    uint8_t homing_start_in = false;
    if (port_homing_start_in_.read(homing_start_in) == RTT::NewData && homing_start_in) {
//        Logger::log() << Logger::Info << getName() << " received homing_start command" << Logger::endl;
        if (servo_state_ == ServoState::OPERATION_ENABLED) {
          homing_ = true;
        }
    }

  if (homing_) {
    if ((statusWord_in & 0x3400) == 0x1400) {
      homing_ = false;
      homing_done_ = true;
    }
  }

  if (homing_done_) {
    port_modeOfOperation_out_.write(control_mode_);
    atribute_homing_required_ = false;
    atribute_homing_in_progress_ = false;
  } else {
    port_modeOfOperation_out_.write(HOMING);
    if (homing_) {
      atribute_homing_in_progress_ = true;
      controlWord_out |= 0x10;
    }
    else {
      atribute_homing_in_progress_ = false;
    }
  }

  if (servo_state_ != ServoState::OPERATION_ENABLED || homing_done_ == false) {
    switch (control_mode_) {
      case CYCLIC_CURRENT:
        port_i_out_.write(0);
        break;
      case CYCLIC_VELOCITY:
        port_dq_out_.write(0);
        break;
      case CYCLIC_POSITION:
        port_q_out_.write(q_in);
        break;
      default:
        break;
    }
  } else {
    switch (control_mode_) {
      case CYCLIC_CURRENT:
        int16_t i_in;
        if (port_desired_i_in_.read(i_in) == RTT::NewData) {
          port_i_out_.write(i_in);
        }
        else {
          port_i_out_.write(0);
        }
        break;
      case CYCLIC_VELOCITY:
        int32_t dq_in;
        if (port_desired_dq_in_.read(dq_in) == RTT::NewData) {
          port_dq_out_.write(dq_in);
        }
        break;
      case CYCLIC_POSITION:
        int32_t q_in;
        if (port_desired_q_in_.read(q_in) == RTT::NewData) {
          port_q_out_.write(q_in);
        }
        break;
      default:
        break;
    }
  }

  port_controlWord_out_.write(controlWord_out);

  if (servo_state_ == ServoState::OPERATION_ENABLED) {
      port_enabled_out_.write(1);
  }
  else {
      port_enabled_out_.write(0);
  }

  port_homing_required_out_.write(atribute_homing_required_);
  port_homing_in_progress_out_.write(atribute_homing_in_progress_);

  if (enable_prev != enable_) {
     Logger::log() << Logger::Info << getName() << " enable: " << (enable_?"t":"f") << Logger::endl;
  }
}

}   // velma_core_ve_body_types

ORO_LIST_COMPONENT_TYPE(velma_core_ve_body_types::ElmoDriver)

