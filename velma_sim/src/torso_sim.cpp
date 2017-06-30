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

class TorsoSim : public RTT::TaskContext
{
public:

    // torso ports
    RTT::InputPort<int16_t >    port_t_MotorCurrentCommand_in_;
    RTT::OutputPort<int32_t >   port_t_MotorPosition_out_;
    RTT::OutputPort<int32_t >   port_t_MotorVelocity_out_;

    double t_t_;
    double q_t_;
    double dq_t_;

    // head ports
    RTT::InputPort<int32_t>      port_hp_q_in_;
    RTT::InputPort<int32_t>      port_hp_v_in_;
    RTT::InputPort<int32_t>      port_hp_c_in_;
    RTT::OutputPort<int32_t>     port_hp_q_out_;
    RTT::OutputPort<int32_t>     port_hp_v_out_;
    RTT::InputPort<int32_t>      port_ht_q_in_;
    RTT::InputPort<int32_t>      port_ht_v_in_;
    RTT::InputPort<int32_t>      port_ht_c_in_;
    RTT::OutputPort<int32_t>     port_ht_q_out_;
    RTT::OutputPort<int32_t>     port_ht_v_out_;

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

    // public methods
    TorsoSim(std::string const& name);
    ~TorsoSim();
    void updateHook();
    bool startHook();
    bool configureHook();
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
    {
        // torso ports
        this->ports()->addPort(port_t_MotorCurrentCommand_in_);
        this->ports()->addPort(port_t_MotorPosition_out_);
        this->ports()->addPort(port_t_MotorVelocity_out_);

        // head ports
        this->ports()->addPort("head_pan_motor_position_command_INPORT",        port_hp_q_in_);
        this->ports()->addPort("head_pan_motor_velocity_command_INPORT",        port_hp_v_in_);
        this->ports()->addPort("head_pan_motor_current_command_INPORT",         port_hp_c_in_);
        this->ports()->addPort(port_hp_q_out_);
        this->ports()->addPort(port_hp_v_out_);
        hp_q_in_ = hp_v_in_ = hp_c_in_ = hp_q_out_ = hp_v_out_ = 0.0;
        this->ports()->addPort("head_tilt_motor_position_command_INPORT",       port_ht_q_in_);
        this->ports()->addPort("head_tilt_motor_velocity_command_INPORT",       port_ht_v_in_);
        this->ports()->addPort("head_tilt_motor_current_command_INPORT",        port_ht_c_in_);
        this->ports()->addPort(port_ht_q_out_);
        this->ports()->addPort(port_ht_v_out_);
        ht_q_in_ = ht_v_in_ = ht_c_in_ = ht_q_out_ = ht_v_out_ = 0.0;
    }

    TorsoSim::~TorsoSim() {
    }

    void TorsoSim::updateHook() {
        const double torso_gear = 158.0;
        const double torso_trans_mult = 131072.0 * torso_gear / (M_PI * 2.0);
        const double torso_motor_offset = 270119630.0;
        const double torso_joint_offset = 0;
        const double torso_motor_constant = 0.00105;

        int16_t t_MotorCurrentCommand_in;
        if (port_t_MotorCurrentCommand_in_.read(t_MotorCurrentCommand_in) != RTT::NewData) {
            //t_MotorCurrentCommand_in_ = 0;
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
    }

    bool TorsoSim::startHook() {
      return true;
    }

    bool TorsoSim::configureHook() {
        return true;
    }

ORO_LIST_COMPONENT_TYPE(TorsoSim)

