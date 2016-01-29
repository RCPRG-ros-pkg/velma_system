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

#include "velma_gazebo.h"

    VelmaGazebo::VelmaGazebo(std::string const& name) : 
        TaskContext(name)
    {

        nh_ = new ros::NodeHandle();
        std::cout << "VelmaGazebo ROS node namespace: " << nh_->getNamespace() << std::endl;

        std::cout << "VelmaGazebo ROS node name: " << ros::this_node::getName() << std::endl;
        std::cout << "VelmaGazebo ROS node namespace2: " << ros::this_node::getNamespace() << std::endl;

        // Add required gazebo interfaces
        this->provides("gazebo")->addOperation("configure",&VelmaGazebo::gazeboConfigureHook,this,RTT::ClientThread);
        this->provides("gazebo")->addOperation("update",&VelmaGazebo::gazeboUpdateHook,this,RTT::ClientThread);

        // right KUKA FRI ports
        this->ports()->addPort("r_JointTorqueCommand_INPORT",        port_r_JointTorqueCommand_in_).doc("");
        this->ports()->addPort("r_KRL_CMD_INPORT",                   port_r_KRL_CMD_in_).doc("");
        this->ports()->addPort("r_CartesianWrench_OUTPORT",           port_r_CartesianWrench_out_).doc("");
        this->ports()->addPort("r_RobotState_OUTPORT",                port_r_RobotState_out_).doc("");
        this->ports()->addPort("r_FRIState_OUTPORT",                  port_r_FRIState_out_).doc("");
        this->ports()->addPort("r_JointVelocity_OUTPORT",             port_r_JointVelocity_out_).doc("");
        this->ports()->addPort("r_MassMatrix_OUTPORT",                port_r_MassMatrix_out_).doc("");
        this->ports()->addPort("r_JointTorque_OUTPORT",               port_r_JointTorque_out_).doc("");
        this->ports()->addPort("r_GravityTorque_OUTPORT",             port_r_GravityTorque_out_);
        this->ports()->addPort("r_JointPosition_OUTPORT",             port_r_JointPosition_out_).doc("");
        r_JointTorqueCommand_in_.resize(7);
        r_JointTorqueCommand_in_.setZero();
        r_JointPosition_out_.resize(7);
        r_JointVelocity_out_.resize(7);
        r_JointTorque_out_.resize(7);
        r_GravityTorque_out_.resize(7);
        port_r_JointPosition_out_.setDataSample(    r_JointPosition_out_);
        port_r_JointVelocity_out_.setDataSample(    r_JointVelocity_out_);
        port_r_JointTorque_out_.setDataSample(      r_JointTorque_out_);
        port_r_GravityTorque_out_.setDataSample(    r_GravityTorque_out_);

        // left KUKA FRI ports
        this->ports()->addPort("l_JointTorqueCommand_INPORT",         port_l_JointTorqueCommand_in_).doc("");
        this->ports()->addPort("l_KRL_CMD_INPORT",                    port_l_KRL_CMD_in_).doc("");
        this->ports()->addPort("l_CartesianWrench_OUTPORT",           port_l_CartesianWrench_out_).doc("");
        this->ports()->addPort("l_RobotState_OUTPORT",                port_l_RobotState_out_).doc("");
        this->ports()->addPort("l_FRIState_OUTPORT",                  port_l_FRIState_out_).doc("");
        this->ports()->addPort("l_JointVelocity_OUTPORT",             port_l_JointVelocity_out_).doc("");
        this->ports()->addPort("l_MassMatrix_OUTPORT",                port_l_MassMatrix_out_).doc("");
        this->ports()->addPort("l_JointTorque_OUTPORT",               port_l_JointTorque_out_).doc("");
        this->ports()->addPort("l_GravityTorque_OUTPORT",             port_l_GravityTorque_out_);
        this->ports()->addPort("l_JointPosition_OUTPORT",             port_l_JointPosition_out_).doc("");
        l_JointTorqueCommand_in_.resize(7);
        l_JointTorqueCommand_in_.setZero();
        l_JointPosition_out_.resize(7);
        l_JointVelocity_out_.resize(7);
        l_JointTorque_out_.resize(7);
        l_GravityTorque_out_.resize(7);
        port_l_JointPosition_out_.setDataSample(    l_JointPosition_out_);
        port_l_JointVelocity_out_.setDataSample(    l_JointVelocity_out_);
        port_l_JointTorque_out_.setDataSample(      l_JointTorque_out_);
        port_l_GravityTorque_out_.setDataSample(    l_GravityTorque_out_);

        // torso ports
        this->ports()->addPort("t_MotorCurrentCommand_INPORT",        port_t_MotorCurrentCommand_in_).doc("");
        this->ports()->addPort("t_MotorPosition_OUTPORT",             port_t_MotorPosition_out_).doc("");
        this->ports()->addPort("t_MotorVelocity_OUTPORT",             port_t_MotorVelocity_out_).doc("");
        t_MotorCurrentCommand_in_ = 0.0;

        r_command_mode_ = false;
        l_command_mode_ = false;

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

    VelmaGazebo::~VelmaGazebo() {
    }

ORO_LIST_COMPONENT_TYPE(VelmaGazebo)
ORO_CREATE_COMPONENT_LIBRARY();

