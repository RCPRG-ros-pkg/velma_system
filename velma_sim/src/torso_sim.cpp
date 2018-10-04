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

#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>

#include <controller_common/elmo_servo_state.h>

using namespace controller_common::elmo_servo;
using namespace RTT;

class TorsoSim : public RTT::TaskContext
{
public:

    // torso ports
    RTT::InputPort<int16_t >    port_t_MotorCurrentCommand_in_;
    RTT::InputPort<uint16_t >   port_t_MotorControlWord_in_;
    RTT::OutputPort<int32_t >   port_t_MotorPosition_out_;
    RTT::OutputPort<int32_t >   port_t_MotorVelocity_out_;
    RTT::OutputPort<uint16_t >  port_t_MotorStatus_out_;

    int16_t t_MotorCurrentCommand_in_;
    int32_t t_MotorPosition_out_;
    int32_t t_MotorVelocity_out_;

    // head ports
    RTT::InputPort<int32_t>      port_hp_q_in_;
    RTT::InputPort<int32_t>      port_hp_v_in_;
    RTT::InputPort<int32_t>      port_hp_c_in_;
    RTT::InputPort<uint16_t >    port_hp_controlWord_in_;
    RTT::OutputPort<int32_t>     port_hp_q_out_;
    RTT::OutputPort<int32_t>     port_hp_v_out_;
    RTT::OutputPort<uint16_t >   port_hp_status_out_;

    RTT::InputPort<int32_t>      port_ht_q_in_;
    RTT::InputPort<int32_t>      port_ht_v_in_;
    RTT::InputPort<int32_t>      port_ht_c_in_;
    RTT::InputPort<uint16_t >    port_ht_controlWord_in_;
    RTT::OutputPort<int32_t>     port_ht_q_out_;
    RTT::OutputPort<int32_t>     port_ht_v_out_;
    RTT::OutputPort<uint16_t >   port_ht_status_out_;

    int32_t hp_q_in_;
    int32_t hp_v_in_;
    int32_t hp_c_in_;
    int32_t hp_q_out_;
    int32_t hp_v_out_;
    int32_t ht_q_in_;
    int32_t ht_v_in_;
    int32_t ht_c_in_;
    int32_t ht_q_out_;
    int32_t ht_v_out_;

    bool hp_homing_done_;
    bool hp_homing_in_progress_;

    bool ht_homing_done_;
    bool ht_homing_in_progress_;

    controller_common::elmo_servo::ServoState t_servo_state_;
    controller_common::elmo_servo::ServoState hp_servo_state_;
    controller_common::elmo_servo::ServoState ht_servo_state_;

    double q_t_;
    double dq_t_;
    double t_t_;

    double q_hp_;
    double q_ht_;

    // public methods
    TorsoSim(std::string const& name);
    ~TorsoSim();
    void updateHook();
    bool startHook();
    bool configureHook();

    ServoState getNextServoState(ServoState current_state, uint16_t controlWord) const;
};

using namespace RTT;

    TorsoSim::TorsoSim(std::string const& name)
        : TaskContext(name, RTT::TaskContext::PreOperational)
        , port_t_MotorCurrentCommand_in_("t_MotorCurrentCommand_INPORT")
        , port_t_MotorPosition_out_("t_MotorPosition_OUTPORT", false)
        , port_t_MotorVelocity_out_("t_MotorVelocity_OUTPORT", false)
        , port_hp_q_out_("head_pan_motor_position_OUTPORT", false)
        , port_hp_v_out_("head_pan_motor_velocity_OUTPORT", false)
        , port_ht_q_out_("head_tilt_motor_position_OUTPORT", false)
        , port_ht_v_out_("head_tilt_motor_velocity_OUTPORT", false)
        , q_t_(0.0)
        , dq_t_(0.0)
        , t_t_(0.0)
        , q_hp_(0.0)
        , q_ht_(0.0)
        , hp_homing_done_(false)
        , hp_homing_in_progress_(false)
        , ht_homing_done_(false)
        , ht_homing_in_progress_(false)
        , t_servo_state_(ServoState::NOT_READY_TO_SWITCH_ON)
        , hp_servo_state_(ServoState::NOT_READY_TO_SWITCH_ON)
        , ht_servo_state_(ServoState::NOT_READY_TO_SWITCH_ON)
    {
        // torso ports
        this->ports()->addPort("t_MotorControlWord_INPORT",         port_t_MotorControlWord_in_);
        this->ports()->addPort(port_t_MotorCurrentCommand_in_);
        this->ports()->addPort(port_t_MotorPosition_out_);
        this->ports()->addPort(port_t_MotorVelocity_out_);
        this->ports()->addPort("t_MotorStatus_OUTPORT", port_t_MotorStatus_out_);

        // head ports
        this->ports()->addPort("head_pan_motor_position_command_INPORT",        port_hp_q_in_);
        this->ports()->addPort("head_pan_motor_velocity_command_INPORT",        port_hp_v_in_);
        this->ports()->addPort("head_pan_motor_current_command_INPORT",         port_hp_c_in_);
        this->ports()->addPort(port_hp_q_out_);
        this->ports()->addPort(port_hp_v_out_);
        this->ports()->addPort("head_pan_motor_controlWord_INPORT",             port_hp_controlWord_in_);
        this->ports()->addPort("head_pan_motor_status_OUTPORT", port_hp_status_out_);
        hp_q_in_ = hp_v_in_ = hp_c_in_ = hp_q_out_ = hp_v_out_ = 0.0;
        this->ports()->addPort("head_tilt_motor_position_command_INPORT",       port_ht_q_in_);
        this->ports()->addPort("head_tilt_motor_velocity_command_INPORT",       port_ht_v_in_);
        this->ports()->addPort("head_tilt_motor_current_command_INPORT",        port_ht_c_in_);
        this->ports()->addPort(port_ht_q_out_);
        this->ports()->addPort(port_ht_v_out_);
        this->ports()->addPort("head_tilt_motor_controlWord_INPORT",            port_ht_controlWord_in_);
        this->ports()->addPort("head_tilt_motor_status_OUTPORT", port_ht_status_out_);
        ht_q_in_ = ht_v_in_ = ht_c_in_ = ht_q_out_ = ht_v_out_ = 0.0;
    }

    TorsoSim::~TorsoSim() {
    }

    ServoState TorsoSim::getNextServoState(ServoState current_state, uint16_t controlWord) const {
        ServoState next_state;
        if ( current_state == ServoState::NOT_READY_TO_SWITCH_ON && (controlWord&0x000F) == 0x0000 ) {
            return ServoState::SWITCH_ON_DISABLED;
        }
        else if ( current_state == ServoState::SWITCH_ON_DISABLED && (controlWord&0x000F) == 0x0006 ) {
            return ServoState::READY_TO_SWITCH_ON;
        }
        else if ( current_state == ServoState::READY_TO_SWITCH_ON && (controlWord&0x000F) == 0x0007 ) {
            return ServoState::SWITCH_ON;
        }
        else if ( current_state == ServoState::SWITCH_ON && (controlWord&0x000F) == 0x000F ) {
            return ServoState::OPERATION_ENABLED;
        }
        else if ( current_state == ServoState::OPERATION_ENABLED && (controlWord&0x000F) == 0x0007 ) {
            return ServoState::SWITCH_ON;
        }
        return current_state;
    }

    void TorsoSim::updateHook() {
        const double torso_gear = 158.0;
        const double torso_trans_mult = 131072.0 * torso_gear / (M_PI * 2.0);
        const double torso_motor_offset = 270119630.0;
        const double torso_joint_offset = 0;
        const double torso_motor_constant = 0.00105;

        int16_t t_MotorCurrentCommand_in = 0;
        if (port_t_MotorCurrentCommand_in_.read(t_MotorCurrentCommand_in) != RTT::NewData) {
            Logger::log() << Logger::Info << "could not read torso motor torque" << Logger::endl;
        }

        t_t_ = t_MotorCurrentCommand_in * torso_gear * torso_motor_constant;

        dq_t_ += t_t_ * 0.001;

        // damping
        dq_t_ *= 0.99;

        // integrate velocity
        q_t_ += dq_t_ * 0.001;

        int32_t t_MotorPosition_out = (q_t_ - torso_joint_offset) * torso_trans_mult + torso_motor_offset;
        int32_t t_MotorVelocity_out = dq_t_ * torso_trans_mult;

        port_t_MotorPosition_out_.write(t_MotorPosition_out);
        port_t_MotorVelocity_out_.write(t_MotorVelocity_out);

        //
        // head
        //
        port_hp_q_in_.read(hp_q_in_);
        port_hp_v_in_.read(hp_v_in_);
        port_hp_c_in_.read(hp_c_in_);
        port_hp_q_out_.write(hp_q_out_);
        port_hp_v_out_.write(hp_v_out_);
        port_ht_q_in_.read(ht_q_in_);
        port_ht_v_in_.read(ht_v_in_);
        port_ht_c_in_.read(ht_c_in_);
        port_ht_q_out_.write(ht_q_out_);
        port_ht_v_out_.write(ht_v_out_);

        const double head_trans = 8000.0 * 100.0 / (M_PI * 2.0);

        if (hp_homing_in_progress_) {
            if (q_hp_ > 0.1) {
                q_hp_ -= 0.0005;
            }
            else if (q_hp_ < -0.1) {
                q_hp_ += 0.0005;
            }
            else {
                hp_homing_in_progress_ = false;
                hp_homing_done_ = true;
            }
        }
        else {
            q_hp_ = hp_q_in_ / head_trans;
        }
        hp_q_out_ = q_hp_ * head_trans;

        if (ht_homing_in_progress_) {
            if (q_ht_ > 0.1) {
                q_ht_ -= 0.0005;
            }
            else if (q_ht_ < -0.1) {
                q_ht_ += 0.0005;
            }
            else {
                ht_homing_in_progress_ = false;
                ht_homing_done_ = true;
            }
        }
        else {
            q_ht_ = ht_q_in_ / head_trans;
        }
        ht_q_out_ = q_ht_ * head_trans;




        // copied from velma_sim_gazebo/src/torso_gazebo_orocos.cpp
        uint16_t hp_controlWord_in;
        if (port_hp_controlWord_in_.read(hp_controlWord_in) == RTT::NewData) {
            if ( (hp_controlWord_in&0x10) != 0 && !hp_homing_in_progress_) {
                if (hp_homing_done_) {
                    Logger::log() << Logger::Info << "Running homing second time for head pan motor!" << Logger::endl;
                }
                else {
                    hp_homing_in_progress_ = true;
                    Logger::log() << Logger::Info << "Running homing head pan motor" << Logger::endl;
                }
            }
            ServoState prev_state = hp_servo_state_;
            hp_servo_state_ = getNextServoState(hp_servo_state_, hp_controlWord_in);
            if (prev_state != hp_servo_state_) {
                Logger::log() << Logger::Info << "hp motor state: " << getServoStateStr(hp_servo_state_) << Logger::endl;
            }
        }
        else {
            hp_servo_state_ = ServoState::NOT_READY_TO_SWITCH_ON;
            Logger::log() << Logger::Info << "hp motor state: " << getServoStateStr(hp_servo_state_) << Logger::endl;
        }

        uint16_t ht_controlWord_in;
        if (port_ht_controlWord_in_.read(ht_controlWord_in) == RTT::NewData) {
            if ( (ht_controlWord_in&0x10) != 0 && !ht_homing_in_progress_) {
                if (ht_homing_done_) {
                    Logger::log() << Logger::Info << "Running homing second time for head tilt motor!" << Logger::endl;
                }
                else {
                    ht_homing_in_progress_ = true;
                    Logger::log() << Logger::Info << "Running homing head tilt motor" << Logger::endl;
                }
            }
            ServoState prev_state = ht_servo_state_;
            ht_servo_state_ = getNextServoState(ht_servo_state_, ht_controlWord_in);
            if (prev_state != ht_servo_state_) {
                Logger::log() << Logger::Info << "ht motor state: " << getServoStateStr(ht_servo_state_) << Logger::endl;
            }
        }
        else {
            ht_servo_state_ = ServoState::NOT_READY_TO_SWITCH_ON;
            Logger::log() << Logger::Info << "ht motor state: " << getServoStateStr(ht_servo_state_) << Logger::endl;
        }

        uint16_t t_controlWord_in;
        if (port_t_MotorControlWord_in_.read(t_controlWord_in) == RTT::NewData) {
            ServoState prev_state = t_servo_state_;
            t_servo_state_ = getNextServoState(t_servo_state_, t_controlWord_in);
            if (prev_state != t_servo_state_) {
                Logger::log() << Logger::Info << "t motor state: " << getServoStateStr(t_servo_state_) << Logger::endl;
            }
        }
        else {
            t_servo_state_ = ServoState::NOT_READY_TO_SWITCH_ON;
            Logger::log() << Logger::Info << "t motor state: " << getServoStateStr(t_servo_state_) << Logger::endl;
        }

        uint16_t t_status_out = getStatusWord(t_servo_state_);
        uint16_t hp_status_out = getStatusWord(hp_servo_state_);
        uint16_t ht_status_out = getStatusWord(ht_servo_state_);

        if (hp_homing_done_) {
            hp_status_out |= 0x1400;
        }
        if (ht_homing_done_) {
            ht_status_out |= 0x1400;
        }

        port_t_MotorStatus_out_.write(t_status_out);
        port_hp_status_out_.write(hp_status_out);
        port_ht_status_out_.write(ht_status_out);
    }

    bool TorsoSim::startHook() {
      return true;
    }

    bool TorsoSim::configureHook() {
        return true;
    }

ORO_LIST_COMPONENT_TYPE(TorsoSim)

