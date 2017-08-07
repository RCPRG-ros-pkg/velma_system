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

    int control_mode_;

    void calculateDampingTorque(int32_t motor_velocity, int16_t &motor_current_command);

    // ROS parameters
    std::string control_mode_str_;

	double damping_factor_;
    double gear_;
    double encoder_resolution_;
    double motor_constant_;

    // i/o ports
    RTT::InputPort<int32_t > port_q_in_;
    RTT::InputPort<int32_t > port_dq_in_;

    RTT::OutputPort<int16_t > port_desired_i_out_;
    RTT::OutputPort<int32_t > port_desired_q_out_;
    RTT::OutputPort<int32_t > port_desired_dq_out_;

    RTT::InputPort<uint32_t > port_cycle_counter_in_;
    RTT::OutputPort<uint8_t > port_disable_out_;

    int32_t q_in_;
};

SafeElmo::SafeElmo(const std::string &name)
    : TaskContext(name, PreOperational)
    , damping_factor_(-1)   // initialize with invalid value, should be later set to >= 0
    , gear_(0.0)
    , encoder_resolution_(0.0)
    , motor_constant_(0.0)
    , control_mode_(HOMING)
{
    addProperty("control_mode", control_mode_str_);
    addProperty("damping_factor", damping_factor_);
    addProperty("gear", gear_);
    addProperty("encoder_resolution", encoder_resolution_);
    addProperty("motor_constant", motor_constant_);

    this->ports()->addPort("disable_OUTPORT", port_disable_out_);
}

bool SafeElmo::configureHook() {
    Logger::In in( std::string("SafeElmo::configureHook ") + getName());

    if (control_mode_str_ == "position") {
        this->ports()->addPort("q_INPORT", port_q_in_);
        this->ports()->addPort("q_OUTPORT", port_desired_q_out_);
        this->ports()->addPort("cycle_counter_INPORT", port_cycle_counter_in_);
        control_mode_ = CYCLIC_POSITION;
        Logger::log() << Logger::Info << "Control mode is set to '" << control_mode_str_ << "'" << Logger::endl;
    }
    else if (control_mode_str_ == "velocity") {
        this->ports()->addPort("dq_OUTPORT", port_desired_dq_out_);
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
        this->ports()->addPort("dq_INPORT", port_dq_in_);
        this->ports()->addPort("i_OUTPORT", port_desired_i_out_);
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

    uint8_t disable_out = true;
    port_disable_out_.write(disable_out);

    if (control_mode_ == CYCLIC_CURRENT) {
        int32_t dq_in;
        if (port_dq_in_.read(dq_in) == RTT::NewData) {
            int16_t i;
            calculateDampingTorque(dq_in, i);
            port_desired_i_out_.write(i);
        }
        else {
            port_desired_i_out_.write(0);
        }
    }
    else if (control_mode_ == CYCLIC_VELOCITY) {
        port_desired_dq_out_.write(0);
    }
    else if (control_mode_ == CYCLIC_POSITION) {
        uint32_t cycle_counter = 0;
        if (port_cycle_counter_in_.read(cycle_counter) != RTT::NewData) {
            Logger::In in( std::string("SafeElmo::updateHook ") + getName());
            Logger::log() << Logger::Error << "could not read port " << port_cycle_counter_in_.getName() << Logger::endl;
            error();
            return;
        }

        if (cycle_counter < 3) {
            if (port_q_in_.read(q_in_) != RTT::NewData) {
                //TODO: error?
            }
        }
        port_desired_q_out_.write(q_in_);
    }
    else {
        Logger::In in( std::string("SafeElmo::updateHook ") + getName());
        Logger::log() << Logger::Error << "wrong mode of operation: " << control_mode_ << Logger::endl;
        error();
    }
}

}   // velma_core_ve_body_types

ORO_LIST_COMPONENT_TYPE(velma_core_ve_body_types::SafeElmo)

