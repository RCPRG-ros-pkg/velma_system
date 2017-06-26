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

#include <std_msgs/Int32.h>

#include "Eigen/Dense"

#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>

#include <geometry_msgs/Wrench.h>

#include <lwr_msgs/FriRobotState.h>
#include <lwr_msgs/FriIntfState.h>

typedef Eigen::Matrix<double, 7, 7> Matrix77d;

class LWRSim : public RTT::TaskContext
{
protected:
    typedef Eigen::Matrix<double, 7, 1> Joints;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // right KUKA FRI ports
    RTT::InputPort<Joints >                 port_JointTorqueCommand_in_;  // FRIx.JointTorqueCommand
    RTT::InputPort<std_msgs::Int32 >        port_KRL_CMD_in_;             // FRIx.KRL_CMD
    RTT::OutputPort<lwr_msgs::FriRobotState >   port_RobotState_out_;     // FRIx.RobotState
    RTT::OutputPort<lwr_msgs::FriIntfState >    port_FRIState_out_;       // FRIx.FRIState
    RTT::OutputPort<Joints >                port_JointPosition_out_;      // FRIx.JointPosition
    RTT::OutputPort<Joints >                port_JointVelocity_out_;      // FRIx.JointVelocity
    RTT::OutputPort<geometry_msgs::Wrench > port_CartesianWrench_out_;    // FRIx.CartesianWrench
    RTT::OutputPort<Matrix77d >             port_MassMatrix_out_;         // FRIx.MassMatrix
    RTT::OutputPort<Joints >                port_JointTorque_out_;        // FRIx.JointTorque
    RTT::OutputPort<Joints >                port_GravityTorque_out_;      // FRIx.GravityTorque

    Joints                  JointTorqueCommand_in_;
    std_msgs::Int32         KRL_CMD_in_;
    lwr_msgs::FriRobotState RobotState_out_;
    lwr_msgs::FriIntfState  FRIState_out_;
    Joints                  JointPosition_out_;
    Joints                  JointVelocity_out_;
    geometry_msgs::Wrench   CartesianWrench_out_;
    Matrix77d               MassMatrix_out_;
    Joints                  JointTorque_out_;
    Joints                  GravityTorque_out_;

    // public methods
    LWRSim(std::string const& name);
    ~LWRSim();
    void updateHook();
    bool startHook();
    bool configureHook();

  protected:

    // ROS parameters
	std::vector<double> init_joint_positions_;

    // torso and KUKA LWRs
    bool command_mode_;
};

using namespace RTT;

    LWRSim::LWRSim(std::string const& name)
        : TaskContext(name, RTT::TaskContext::PreOperational)
        , port_CartesianWrench_out_("CartesianWrench_OUTPORT", false)
        , port_RobotState_out_("RobotState_OUTPORT", false)
        , port_FRIState_out_("FRIState_OUTPORT", false)
        , port_JointVelocity_out_("JointVelocity_OUTPORT", false)
        , port_MassMatrix_out_("MassMatrix_OUTPORT", false)
        , port_JointTorque_out_("JointTorque_OUTPORT", false)
        , port_GravityTorque_out_("GravityTorque_OUTPORT", false)
        , port_JointPosition_out_("JointPosition_OUTPORT", false)
    {
        addProperty("init_joint_positions", init_joint_positions_);

        // right KUKA FRI ports
        this->ports()->addPort("JointTorqueCommand_INPORT",         port_JointTorqueCommand_in_).doc("");
        this->ports()->addPort("KRL_CMD_INPORT",                    port_KRL_CMD_in_).doc("");
        this->ports()->addPort(port_CartesianWrench_out_);
        this->ports()->addPort(port_RobotState_out_);
        this->ports()->addPort(port_FRIState_out_);
        this->ports()->addPort(port_JointVelocity_out_);
        this->ports()->addPort(port_MassMatrix_out_);
        this->ports()->addPort(port_JointTorque_out_);
        this->ports()->addPort(port_GravityTorque_out_);
        this->ports()->addPort(port_JointPosition_out_);

        command_mode_ = false;
    }

    LWRSim::~LWRSim() {
    }

    void LWRSim::updateHook() {

        if (port_KRL_CMD_in_.read(KRL_CMD_in_) == RTT::NewData) {
            if (1 == KRL_CMD_in_.data) {
                if (!command_mode_) {
                    command_mode_ = true;
                    //Logger::log() << Logger::Info <<  "switched to command mode" << Logger::endl;
                }
                else {
                    //Logger::log() << Logger::Warning <<  "tried to switch to command mode while in command mode" << Logger::endl;
                }
            }
            else if (2 == KRL_CMD_in_.data) {
                if (command_mode_) {
                    command_mode_ = false;
                    //Logger::log() << Logger::Info << "switched to monitor mode" << Logger::endl;
                }
                else {
                    //Logger::log() << Logger::Warning <<  "tried to switch to monitor mode while in monitor mode" << Logger::endl;
                }
            }
        }

        if (port_JointTorqueCommand_in_.read(JointTorqueCommand_in_) == RTT::NewData) {
        }

        for (int i = 0; i < 7; ++i) {
            // integrate torque
            // the mass matrix is identity, so use only time factor (0.001)
            JointVelocity_out_(i) += JointTorqueCommand_in_(i) * 0.001;

            // damping
            JointVelocity_out_(i) *= 0.99;

            // integrate velocity
            JointPosition_out_(i) += JointVelocity_out_(i) * 0.001;
        }

        // FRI comm state
        FRIState_out_.quality = lwr_msgs::FriIntfState::FRI_QUALITY_PERFECT;
        if (command_mode_) {
            FRIState_out_.state = lwr_msgs::FriIntfState::FRI_STATE_CMD;
        }
        else {
            FRIState_out_.state = lwr_msgs::FriIntfState::FRI_STATE_MON;
        }
        port_FRIState_out_.write(FRIState_out_);
        port_FRIState_out_.write(FRIState_out_);

        // FRI robot state
        RobotState_out_.power = 0x7F;
        RobotState_out_.error = 0;
        RobotState_out_.warning = 0;
        RobotState_out_.control = lwr_msgs::FriRobotState::FRI_CTRL_JNT_IMP;
        port_RobotState_out_.write(RobotState_out_);

        port_CartesianWrench_out_.write(CartesianWrench_out_);

        port_MassMatrix_out_.write(MassMatrix_out_);
        port_GravityTorque_out_.write(GravityTorque_out_);
        port_JointTorque_out_.write(JointTorque_out_);
        port_JointPosition_out_.write(JointPosition_out_);
        port_JointVelocity_out_.write(JointVelocity_out_);
    }

    bool LWRSim::startHook() {
      return true;
    }

    bool LWRSim::configureHook() {
        Logger::In in("LWRSim::configureHook");

        JointTorqueCommand_in_.setZero();

        MassMatrix_out_.setZero();
        for (int i = 0; i < MassMatrix_out_.rows(); ++i) {
            MassMatrix_out_(i,i) = 1.0;
        }

        GravityTorque_out_.setZero();
        JointTorque_out_.setZero();
        JointVelocity_out_.setZero();

        if (init_joint_positions_.size() != 7) {
            Logger::log() << Logger::Error <<
                "ROS parameter init_joint_positions has wrong size: " <<
                init_joint_positions_.size() << ", should be 7" << Logger::endl;
            return false;
        }

        for (int i = 0; i < 7; ++i) {
            JointPosition_out_(i) = init_joint_positions_[i];
        }

        return true;
    }

ORO_LIST_COMPONENT_TYPE(LWRSim)

