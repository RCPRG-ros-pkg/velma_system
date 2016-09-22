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

#include "lwr_gazebo.h"

    LWRGazebo::LWRGazebo(std::string const& name) : 
        TaskContext(name),
        data_valid_(false)
    {

        nh_ = new ros::NodeHandle();

        addProperty("init_joint_names", init_joint_names_);
        addProperty("init_joint_positions", init_joint_positions_);
        addProperty("name", name_);
        addProperty("tool", tool_);
//        addProperty("gravity_arm_in_wrist", tool_arm_);
//        addProperty("tool_inertia/ixx", tool_ixx_);
//        addProperty("tool_inertia/ixy", tool_ixy_);
//        addProperty("tool_inertia/ixz", tool_ixz_);
//        addProperty("tool_inertia/iyy", tool_iyy_);
//        addProperty("tool_inertia/iyz", tool_iyz_);
//        addProperty("tool_inertia/izz", tool_izz_);

        // Add required gazebo interfaces
        this->provides("gazebo")->addOperation("configure",&LWRGazebo::gazeboConfigureHook,this,RTT::ClientThread);
        this->provides("gazebo")->addOperation("update",&LWRGazebo::gazeboUpdateHook,this,RTT::ClientThread);

        // right KUKA FRI ports
        this->ports()->addPort("JointTorqueCommand_INPORT",         port_JointTorqueCommand_in_).doc("");
        this->ports()->addPort("KRL_CMD_INPORT",                    port_KRL_CMD_in_).doc("");
        this->ports()->addPort("CartesianWrench_OUTPORT",           port_CartesianWrench_out_).doc("");
        this->ports()->addPort("RobotState_OUTPORT",                port_RobotState_out_).doc("");
        this->ports()->addPort("FRIState_OUTPORT",                  port_FRIState_out_).doc("");
        this->ports()->addPort("JointVelocity_OUTPORT",             port_JointVelocity_out_).doc("");
        this->ports()->addPort("MassMatrix_OUTPORT",                port_MassMatrix_out_).doc("");
        this->ports()->addPort("JointTorque_OUTPORT",               port_JointTorque_out_).doc("");
        this->ports()->addPort("GravityTorque_OUTPORT",             port_GravityTorque_out_);
        this->ports()->addPort("JointPosition_OUTPORT",             port_JointPosition_out_).doc("");
        JointTorqueCommand_in_.resize(7);
        JointTorqueCommand_in_.setZero();
        JointPosition_out_.resize(7);
        JointVelocity_out_.resize(7);
        JointTorque_out_.resize(7);
        GravityTorque_out_.resize(7);
        port_JointPosition_out_.setDataSample(    JointPosition_out_);
        port_JointVelocity_out_.setDataSample(    JointVelocity_out_);
        port_JointTorque_out_.setDataSample(      JointTorque_out_);
        port_GravityTorque_out_.setDataSample(    GravityTorque_out_);

        command_mode_ = false;

        tmp_JointTorqueCommand_in_.resize(7);
        tmp_JointPosition_out_.resize(7);
        tmp_JointVelocity_out_.resize(7);
        tmp_JointTorque_out_.resize(7);
        tmp_GravityTorque_out_.resize(7);
    }

    LWRGazebo::~LWRGazebo() {
    }

ORO_LIST_COMPONENT_TYPE(LWRGazebo)
ORO_CREATE_COMPONENT_LIBRARY();

