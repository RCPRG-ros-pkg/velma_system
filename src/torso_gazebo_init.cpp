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

#include "torso_gazebo.h"

    TorsoGazebo::TorsoGazebo(std::string const& name) : 
        TaskContext(name),
        data_valid_(false)
    {

        nh_ = new ros::NodeHandle();
        std::cout << "TorsoGazebo ROS node namespace: " << nh_->getNamespace() << std::endl;

        std::cout << "TorsoGazebo ROS node name: " << ros::this_node::getName() << std::endl;
        std::cout << "TorsoGazebo ROS node namespace2: " << ros::this_node::getNamespace() << std::endl;

        // Add required gazebo interfaces
        this->provides("gazebo")->addOperation("configure",&TorsoGazebo::gazeboConfigureHook,this,RTT::ClientThread);
        this->provides("gazebo")->addOperation("update",&TorsoGazebo::gazeboUpdateHook,this,RTT::ClientThread);

        // torso ports
        this->ports()->addPort("t_MotorCurrentCommand_INPORT",        port_t_MotorCurrentCommand_in_).doc("");
        this->ports()->addPort("t_MotorPosition_OUTPORT",             port_t_MotorPosition_out_).doc("");
        this->ports()->addPort("t_MotorVelocity_OUTPORT",             port_t_MotorVelocity_out_).doc("");
        t_MotorCurrentCommand_in_ = 0.0;

        // head ports
        this->ports()->addPort("head_pan_motor_position_command_INPORT",        port_hp_q_in_).doc("");
        this->ports()->addPort("head_pan_motor_velocity_command_INPORT",        port_hp_v_in_).doc("");
        this->ports()->addPort("head_pan_motor_current_command_INPORT",         port_hp_c_in_).doc("");
        this->ports()->addPort("head_pan_motor_position_OUTPORT",               port_hp_q_out_).doc("");
        this->ports()->addPort("head_pan_motor_velocity_OUTPORT",               port_hp_v_out_).doc("");
        hp_q_in_ = hp_v_in_ = hp_c_in_ = hp_q_out_ = hp_v_out_ = 0.0;
        this->ports()->addPort("head_tilt_motor_position_command_INPORT",       port_ht_q_in_).doc("");
        this->ports()->addPort("head_tilt_motor_velocity_command_INPORT",       port_ht_v_in_).doc("");
        this->ports()->addPort("head_tilt_motor_current_command_INPORT",        port_ht_c_in_).doc("");
        this->ports()->addPort("head_tilt_motor_position_OUTPORT",              port_ht_q_out_).doc("");
        this->ports()->addPort("head_tilt_motor_velocity_OUTPORT",              port_ht_v_out_).doc("");
        ht_q_in_ = ht_v_in_ = ht_c_in_ = ht_q_out_ = ht_v_out_ = 0.0;
    }

    TorsoGazebo::~TorsoGazebo() {
    }

ORO_LIST_COMPONENT_TYPE(TorsoGazebo)
ORO_CREATE_COMPONENT_LIBRARY();

