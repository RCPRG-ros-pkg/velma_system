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

#include <math.h>

using namespace RTT;

static const uint16_t SW_ReadyToSwitchOn_Mask = 0x0001;
static const uint16_t SW_SwitchedOn_Mask = 0x0002;
static const uint16_t SW_OperationEnabled_Mask = 0x0004;
static const uint16_t SW_Fault_Mask = 0x0008;
static const uint16_t SW_VoltageEnabled_Mask = 0x0010;
static const uint16_t SW_QuickStop_Mask = 0x0020;
static const uint16_t SW_SwitchOnDisabled_Mask = 0x0040;
static const uint16_t SW_Warning_Mask = 0x0080;
static const uint16_t SW_ManufactureSpecific_Mask = 0x0100;
static const uint16_t SW_Remote_Mask = 0x0200;
static const uint16_t SW_TargetReached_Mask = 0x0400;
static const uint16_t SW_InternalLimitActive_Mask = 0x0800;
static const uint16_t SW_HomingComplete_Mask = 0x1000;

static const uint16_t NotReadyToSwitchOn_Mask = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask
    | SW_SwitchOnDisabled_Mask;
static const uint16_t NotReadyToSwitchOn_Pattern = 0x0000;

static const uint16_t SwitchOnDisabled_Mask = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask
    | SW_SwitchOnDisabled_Mask;
static const uint16_t SwitchOnDisabled_Pattern = SW_SwitchOnDisabled_Mask;

static const uint16_t ReadyToSwitchOn_Mask = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask
    | SW_QuickStop_Mask | SW_SwitchOnDisabled_Mask;
static const uint16_t ReadyToSwitchOn_Pattern = SW_ReadyToSwitchOn_Mask
    | SW_QuickStop_Mask;

static const uint16_t SwitchedOn_Mask = SW_ReadyToSwitchOn_Mask | SW_SwitchedOn_Mask
    | SW_OperationEnabled_Mask | SW_Fault_Mask | SW_QuickStop_Mask
    | SW_SwitchOnDisabled_Mask;
static const uint16_t SwitchedOn_Pattern = SW_ReadyToSwitchOn_Mask | SW_SwitchedOn_Mask
    | SW_QuickStop_Mask;

static const uint16_t OperationEnabled_Mask = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask
    | SW_QuickStop_Mask | SW_SwitchOnDisabled_Mask;
static const uint16_t OperationEnabled_Pattern = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_QuickStop_Mask;

static const uint16_t Fault_Reaction_Mask = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask
    | SW_SwitchOnDisabled_Mask;
static const uint16_t Fault_Reaction_Pattern = SW_Fault_Mask | SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask;

static const uint16_t Fault_Mask = SW_ReadyToSwitchOn_Mask | SW_SwitchedOn_Mask
    | SW_OperationEnabled_Mask | SW_Fault_Mask | SW_SwitchOnDisabled_Mask;
static const uint16_t Fault_Pattern = SW_Fault_Mask;

static const uint16_t QuickStopActive_Mask = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask
    | SW_QuickStop_Mask | SW_SwitchOnDisabled_Mask;
static const uint16_t QuickStopActive_Pattern = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask;

static const std::string servoStateNames[] = {
    "INVALID",
    "NOT_READY_TO_SWITCH_ON",
    "SWITCH_ON_DISABLED",
    "READY_TO_SWITCH_ON",
    "SWITCH_ON",
    "OPERATION_ENABLED",
    "QUICK_STOP_ACTIVE",
    "FAULT_REACTION_ACTIVE",
    "FAULT",
};

namespace velma_core_ve_body_types {

class SafeElmo: public RTT::TaskContext {
public:
    explicit SafeElmo(const std::string &name);

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

    enum ServoState {
        INVALID = 0,
        NOT_READY_TO_SWITCH_ON = 1,
        SWITCH_ON_DISABLED = 2,
        READY_TO_SWITCH_ON = 3,
        SWITCH_ON = 4,
        OPERATION_ENABLED = 5,
        QUICK_STOP_ACTIVE = 6,
        FAULT_REACTION_ACTIVE = 7,
        FAULT = 8
    };

    bool enable();
    void disable();
    bool beginHoming();
    bool forceHomingDone();
    bool resetFault();

    int control_mode_;

    ServoState getServoState(uint16_t statusword) const;

    void calculateDampingTorque(int32_t motor_velocity, int16_t &motor_current_command);

    bool isNaN(double d) const;
    bool isInLim(double d, double lo_lim, double hi_lim) const;

    // ROS parameters
    std::string control_mode_str_;

	double damping_factor_;
    double gear_;
    double encoder_resolution_;
    double motor_constant_;

    // ports
    RTT::OutputPort<int16_t > port_i_out_;
    RTT::OutputPort<int32_t > port_q_out_;
    RTT::OutputPort<int32_t > port_dq_out_;
    RTT::OutputPort<uint16_t > port_controlWord_out_;
    RTT::OutputPort<int8_t > port_modeOfOperation_out_;

    RTT::InputPort<int32_t > port_q_in_;
    RTT::InputPort<int32_t > port_dq_in_;
    RTT::InputPort<uint16_t > port_statusWord_in_;

    bool enable_;
    bool homing_;
    bool reset_fault_;
    bool homing_done_;

    ServoState servo_state_;
};

SafeElmo::SafeElmo(const std::string &name)
    : TaskContext(name, PreOperational)
    , damping_factor_(-1)   // initialize with invalid value, should be later set to >= 0
    , gear_(0.0)
    , encoder_resolution_(0.0)
    , motor_constant_(0.0)
    , control_mode_(HOMING)
    , enable_(false)
    , homing_(false)
    , reset_fault_(false)
    , homing_done_(true)
    , servo_state_(INVALID)
{
    this->ports()->addPort("i_OUTPORT", port_i_out_);
    this->ports()->addPort("dq_OUTPORT", port_dq_out_);
    this->ports()->addPort("q_OUTPORT", port_q_out_);
    this->ports()->addPort("controlWord_OUTPORT", port_controlWord_out_);
    this->ports()->addPort("modeOfOperation_OUTPORT", port_modeOfOperation_out_);

    this->ports()->addPort("q_INPORT", port_q_in_);
    this->ports()->addPort("dq_INPORT", port_dq_in_);
    this->ports()->addPort("statusWord_INPORT", port_statusWord_in_);

    addProperty("control_mode", control_mode_str_);
    addProperty("damping_factor", damping_factor_);
    addProperty("gear", gear_);
    addProperty("encoder_resolution", encoder_resolution_);
    addProperty("motor_constant", motor_constant_);
    addProperty("homing_done", homing_done_);

    this->provides()->addOperation("beginHoming", &SafeElmo::beginHoming, this,
                                   RTT::OwnThread);
    this->provides()->addOperation("forceHomingDone",
                                   &SafeElmo::forceHomingDone, this,
                                   RTT::OwnThread);
    this->provides()->addOperation("enable", &SafeElmo::enable, this,
                                   RTT::OwnThread);
    this->provides()->addOperation("disable", &SafeElmo::disable, this,
                                   RTT::OwnThread);
    this->provides()->addOperation("resetFault", &SafeElmo::resetFault, this,
                                   RTT::OwnThread);
}

bool SafeElmo::enable() {
  if (servo_state_ == SWITCH_ON) {
    enable_ = true;
    return true;
  } else {
    return false;
  }
}

void SafeElmo::disable() {
  if (enable_) {
    enable_ = false;
  }
}

bool SafeElmo::beginHoming() {
  if (homing_done_ == false) {
    if (servo_state_ == OPERATION_ENABLED) {
      homing_ = true;
    }
  } else {
    RTT::log(RTT::Error) << "Drive not configured for homing" << RTT::endlog();
  }
  return homing_;
}

bool SafeElmo::forceHomingDone() {
  if (servo_state_ == OPERATION_ENABLED) {
    homing_done_ = true;
  }
  return homing_done_;
}

bool SafeElmo::resetFault() {
  if (servo_state_ == FAULT) {
    reset_fault_ = true;
    return true;
  } else {
    return false;
  }
}

SafeElmo::ServoState SafeElmo::getServoState(uint16_t statusword) const {
  if ((statusword & NotReadyToSwitchOn_Mask) == NotReadyToSwitchOn_Pattern) {
    return NOT_READY_TO_SWITCH_ON;
  } else if ((statusword & SwitchOnDisabled_Mask) == SwitchOnDisabled_Pattern) {
    return SWITCH_ON_DISABLED;
  } else if ((statusword & ReadyToSwitchOn_Mask) == ReadyToSwitchOn_Pattern) {
    return READY_TO_SWITCH_ON;
  } else if ((statusword & SwitchedOn_Mask) == SwitchedOn_Pattern) {
    return SWITCH_ON;
  } else if ((statusword & OperationEnabled_Mask) == OperationEnabled_Pattern) {
    return OPERATION_ENABLED;
  } else if ((statusword & QuickStopActive_Mask) == QuickStopActive_Pattern) {
    return QUICK_STOP_ACTIVE;
  } else if ((statusword & Fault_Reaction_Mask) == Fault_Reaction_Pattern) {
    return FAULT_REACTION_ACTIVE;
  } else if ((statusword & Fault_Mask) == Fault_Pattern) {
    return FAULT;
  } else {
    return INVALID;
  }
}

bool SafeElmo::configureHook() {
    Logger::In in( std::string("SafeElmo::configureHook ") + getName());

    if (control_mode_str_ == "position") {
        control_mode_ = CYCLIC_POSITION;
        Logger::log() << Logger::Info << "Control mode is set to '" << control_mode_str_ << "'" << Logger::endl;
    }
    else if (control_mode_str_ == "velocity") {
        control_mode_ = CYCLIC_VELOCITY;
        Logger::log() << Logger::Info << "Control mode is set to '" << control_mode_str_ << "'" << Logger::endl;
    }
    else if (control_mode_str_ == "torque") {
        if (damping_factor_ < 0) {
            Logger::log() << Logger::Error <<
                "parameter damping_factor is not set or is set to illegal value: " <<
                damping_factor_ << Logger::endl;
            return false;
        }

        if (gear_ == 0.0) {
            Logger::log() << Logger::Error <<
                "parameter gear is not set or is set to illegal value: " <<
                gear_ << Logger::endl;
            return false;
        }

        if (encoder_resolution_ == 0.0) {
            Logger::log() << Logger::Error <<
                "parameter encoder_resolution is not set or is set to illegal value: " <<
                encoder_resolution_ << Logger::endl;
            return false;
        }

        if (motor_constant_ = 0.0) {
            Logger::log() << Logger::Error <<
                "parameter motor_constant is not set or is set to illegal value: " <<
                motor_constant_ << Logger::endl;
            return false;
        }
        control_mode_ = CYCLIC_CURRENT;
        Logger::log() << Logger::Info << "Control mode is set to '" << control_mode_str_ << "'" << Logger::endl;
    }
    else {
        Logger::log() << Logger::Error <<
            "parameter control_mode is not set or is set to illegal value: " <<
            control_mode_str_ << Logger::endl;
        return false;
    }

    return true;
}

bool SafeElmo::startHook() {
    return true;
}

void SafeElmo::stopHook() {
}

void SafeElmo::calculateDampingTorque(int32_t motor_velocity, int16_t &motor_current_command)
{
    const double trans_mult = encoder_resolution_ * gear_ / (M_PI * 2.0);
    double joint_velocity = static_cast<double>(motor_velocity) / trans_mult;
    double motor_torque_command = -damping_factor_ * joint_velocity;
    motor_current_command = static_cast<int16_t>(motor_torque_command / gear_ / motor_constant_);
}

void SafeElmo::updateHook() {
    uint16_t statusWord_in;
    int32_t q_in;
    int32_t dq_in;
    if (port_statusWord_in_.read( statusWord_in ) != RTT::NewData) {
        Logger::log() << Logger::Error << getName() << " could not read statusWord" << Logger::endl;
        // TODO: verify this
        return;
    }
    if (control_mode_ == CYCLIC_POSITION && port_q_in_.read(q_in) != RTT::NewData) {
        Logger::log() << Logger::Error << getName() << " could not read position" << Logger::endl;
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
        Logger::log() << Logger::Info << getName() << " state: " << servoStateNames[servo_state_] << Logger::endl;
    }

    uint16_t controlWord_out = 0;
    switch (servo_state_) {
        case NOT_READY_TO_SWITCH_ON:
          enable_ = false;
          controlWord_out = 0;
          break;
        case SWITCH_ON_DISABLED:
          enable_ = false;
          controlWord_out = 0x06;
          break;
        case READY_TO_SWITCH_ON:
          enable_ = false;
          controlWord_out = 0x07;
          break;
        case SWITCH_ON:
          if (enable_) {
            controlWord_out = 0x0f;
          } else {
            controlWord_out = 0x07;
          }
          break;
        case OPERATION_ENABLED:
          if (enable_) {
            controlWord_out = 0x0f;
          } else {
            controlWord_out = 0x07;
          }
          break;
        case QUICK_STOP_ACTIVE:
          enable_ = false;
          break;
        case FAULT_REACTION_ACTIVE:
          enable_ = false;
          break;
        case FAULT:
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

  if (homing_done_) {
    port_modeOfOperation_out_.write(control_mode_);
  } else {
    port_modeOfOperation_out_.write(HOMING);
    if (homing_) {
      controlWord_out |= 0x10;
    }
  }

  if (servo_state_ != OPERATION_ENABLED || homing_done_ == false) {
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
//        int16_t i;
//        calculateDampingTorque(dq, i);
//        double cur;
//        if (motor_current_command_port_.read(cur) == RTT::NewData) {
          port_i_out_.write(0);
//        }
        break;
      case CYCLIC_VELOCITY:
//        double vel;
//        if (motor_velocity_command_port_.read(vel) == RTT::NewData) {
          port_dq_out_.write(0);
//        }
        break;
      case CYCLIC_POSITION:
//        double pos;
//        if (motor_position_command_port_.read(pos) == RTT::NewData) {
          port_q_out_.write(q_in);
//        }
        break;
      default:
        break;
    }
  }

  port_controlWord_out_.write(controlWord_out);



/*    //
    // read HW state
    //
    if (control_mode_ == CONTROL_MODE_POSITION) {
        int32_t q;
        uint16_t status;
        bool valid = (port_q_in_.read(q) == RTT::NewData);
        valid &= (port_statusWord_in_.read(status) == RTT::NewData);

        if (valid) {
            port_q_out_.write(q);
        }

        ServoState servo_state = getServoState(status);
        
    }
    else if (control_mode_ == CONTROL_MODE_TORQUE) {
        int32_t dq;
        uint16_t status;
        bool valid = (port_dq_in_.read(dq) == RTT::NewData);
        valid &= (port_statusWord_in_.read(status) == RTT::NewData);

        if (valid) {
            int16_t i;
            calculateDampingTorque(dq, i);
            port_i_out_.write(i);
        }
    }
*/
}

}   // velma_core_ve_body_types

ORO_LIST_COMPONENT_TYPE(velma_core_ve_body_types::SafeElmo)

