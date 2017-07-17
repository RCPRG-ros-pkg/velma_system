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

#include <lwr_msgs/FriRobotState.h>
#include <lwr_msgs/FriIntfState.h>

#include <Eigen/Dense>

#include <math.h>

#include "../common_predicates.h"

#include <std_msgs/Int32.h>

using namespace RTT;

namespace velma_core_ve_body_types {

class SafeLWR: public RTT::TaskContext {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit SafeLWR(const std::string &name);

    bool configureHook();

    bool startHook();

    void stopHook();

    void updateHook();

private:

//    typedef Eigen::Matrix<double, 7, 1 > ArmJoints;
    typedef boost::array<double, 7 > ArmJoints;

    const int joints_count_;

    void calculateArmDampingTorque(const ArmJoints &joint_velocity,
        const std::vector<double> &damping_factors, ArmJoints &joint_torque_command);

    bool isNaN(double d) const;
    bool isInLim(double d, double lo_lim, double hi_lim) const;

    // ROS parameters
	std::vector<double> damping_factors_;

    std::vector<double> q_limits_lo_;
    std::vector<double> q_limits_hi_;
    std::vector<double> dq_limits_;
    std::vector<double> t_limits_;

    // ports
    RTT::OutputPort<ArmJoints > port_t_out_;

    RTT::OutputPort<std_msgs::Int32 > port_fri_cmd_out_;

    RTT::InputPort<ArmJoints > port_dq_in_;

    RTT::InputPort<lwr_msgs::FriIntfState > port_fri_in_;
    RTT::InputPort<lwr_msgs::FriRobotState > port_rob_in_;

//    velma_core_cs_ve_body_msgs::StatusSC sc_out_;
//    RTT::OutputPort<velma_core_cs_ve_body_msgs::StatusSC> port_sc_out_;

    ArmJoints dq_;
    lwr_msgs::FriIntfState       fri_state_;
    lwr_msgs::FriRobotState      robot_state_;

//    bool valid_prev_;

//    uint32_t safe_iterations_;
//    bool safe_iterations_over_500_;
};

SafeLWR::SafeLWR(const std::string &name)
    : TaskContext(name, PreOperational)
    , joints_count_(7)
//    , safe_iterations_over_500_(false)
{
    this->ports()->addPort("t_OUTPORT", port_t_out_);
    this->ports()->addPort("fri_OUTPORT", port_fri_cmd_out_);

    this->ports()->addPort("dq_INPORT", port_dq_in_);
    this->ports()->addPort("friIntf_INPORT", port_fri_in_);
    this->ports()->addPort("friRobot_INPORT", port_rob_in_);

//    this->ports()->addPort("sc_OUTPORT", port_sc_out_);

    addProperty("damping_factors", damping_factors_);
    addProperty("q_limits_lo", q_limits_lo_);
    addProperty("q_limits_hi", q_limits_hi_);
    addProperty("dq_limits", dq_limits_);
    addProperty("t_limits", t_limits_);

//    addAttribute("safeIterationsOver500", safe_iterations_over_500_);
}

bool SafeLWR::configureHook() {
    Logger::In in( std::string("SafeLWR::configureHook ") + getName() );

    if (damping_factors_.size() != joints_count_) {
        Logger::log() << Logger::Error <<
            "parameter damping_factors is set to illegal value (wrong vector size: " <<
            damping_factors_.size() << ")" << Logger::endl;
        return false;
    }

    for (int i = 0; i < joints_count_; ++i) {
        if (damping_factors_[i] < 0) {
            Logger::log() << Logger::Error <<
                "parameter damping_factors[" << i << "] is set to illegal value: " <<
                damping_factors_[i] << Logger::endl;
            return false;
        }
    }

    if (q_limits_lo_.size() != joints_count_) {
        Logger::log() << Logger::Error <<
            "parameter q_limits_lo is set to illegal value (wrong vector size: " <<
            q_limits_lo_.size() << ")" << Logger::endl;
        return false;
    }

    if (q_limits_hi_.size() != joints_count_) {
        Logger::log() << Logger::Error <<
            "parameter q_limits_hi is set to illegal value (wrong vector size: " <<
            q_limits_hi_.size() << ")" << Logger::endl;
        return false;
    }

    if (dq_limits_.size() != joints_count_) {
        Logger::log() << Logger::Error <<
            "parameter dq_limits is set to illegal value (wrong vector size: " <<
            dq_limits_.size() << ")" << Logger::endl;
        return false;
    }

    if (t_limits_.size() != joints_count_) {
        Logger::log() << Logger::Error <<
            "parameter t_limits is set to illegal value (wrong vector size: " <<
            t_limits_.size() << ")" << Logger::endl;
        return false;
    }

    return true;
}

bool SafeLWR::startHook() {
//    safe_iterations_ = 0;
//    safe_iterations_over_500_ = false;
//    valid_prev_ = false;
    return true;
}

void SafeLWR::stopHook() {
//    safe_iterations_ = 0;
//    safe_iterations_over_500_ = false;
}

void SafeLWR::calculateArmDampingTorque(const ArmJoints &joint_velocity,
    const std::vector<double> &damping_factors, SafeLWR::ArmJoints &joint_torque_command)
{
    for (int i = 0; i < joints_count_; ++i) {
        joint_torque_command[i] = -damping_factors[i] * joint_velocity[i];
    }
}

void SafeLWR::updateHook() {
    //
    // read HW state
    //
    bool valid = (port_dq_in_.read(dq_) == RTT::NewData);

    valid &= (port_fri_in_.read(fri_state_) == RTT::NewData);
    valid &= (port_rob_in_.read(robot_state_) == RTT::NewData);

    //
    // write commands to available HW devices
    //

    // generate safe outputs for all operational devices
    if (valid) {// || valid_prev_) {
        ArmJoints t;
        calculateArmDampingTorque(dq_, damping_factors_, t);
        port_t_out_.write(t);
    }

    if (valid) {
        bool lwrOk = isLwrOk(robot_state_, fri_state_);

        // try to switch LWR mode
        if (lwrOk && fri_state_.state == lwr_msgs::FriIntfState::FRI_STATE_MON) {
            std_msgs::Int32 cmd;
            cmd.data = 1;
            port_fri_cmd_out_.write(cmd);
        }
    }

    //
    // write diagnostics information for other subsystems
    //
/*    sc_out_.safe_behavior = true;

    sc_out_.fault_type = 0;

    setBit(sc_out_.fault_type, velma_core_cs_ve_body_msgs::StatusSC::FAULT_COMM_HW,
        !(rArm_valid || rArm_valid_prev_) || !(lArm_valid || lArm_valid_prev_) ||
        !torso_valid || !hp_valid ||
        !ht_valid);

    setBit(sc_out_.faulty_module_id, velma_core_cs_ve_body_msgs::StatusSC::MODULE_T_MOTOR,
        !torso_valid);

    setBit(sc_out_.faulty_module_id, velma_core_cs_ve_body_msgs::StatusSC::MODULE_HP_MOTOR,
        !hp_valid);

    setBit(sc_out_.faulty_module_id, velma_core_cs_ve_body_msgs::StatusSC::MODULE_HT_MOTOR,
        !ht_valid);

    sc_out_.error = (sc_out_.fault_type != 0);

// TODO: diagnostics for:
//MODULE_R_HAND
//MODULE_L_HAND
//MODULE_R_FT
//MODULE_L_FT
//MODULE_R_TACTILE
//MODULE_L_OPTOFORCE

    // write diagnostic data
    port_sc_out_.write(sc_out_);

    if (safe_iterations_ < numeric_limits<uint32_t>::max()) {
        ++safe_iterations_;
    }

    if (safe_iterations_ > 500) {
        safe_iterations_over_500_ = true;
    }
    else {
        safe_iterations_over_500_ = false;
    }
*/
//    valid_prev_ = valid;
}

}   // velma_core_ve_body_types

ORO_LIST_COMPONENT_TYPE(velma_core_ve_body_types::SafeLWR)

