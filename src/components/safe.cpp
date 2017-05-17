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

#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>
#include <rtt/base/PortInterface.hpp>

#include <lwr_msgs/FriRobotState.h>
#include <lwr_msgs/FriIntfState.h>

#include "eigen_conversions/eigen_msg.h"

#include "velma_core_cs_ve_body_msgs/StatusSC.h"

#include <math.h>

#include <rtt_rosclock/rtt_rosclock.h>

#include "../common_predicates.h"
#include "velma_core_ve_body/master.h"

#include <std_msgs/Int32.h>
#include <sys/time.h>

using namespace RTT;

static int setBit(int bitfield, int bit_num, bool value) {
    if (value) {
        // set
        bitfield |= (1 << bit_num);
    }
    else {
        // clear
        bitfield &= ~(1 << bit_num);
    }
    return bitfield;
}

static bool getBit(int bitfield, int bit_num) {
    return ((bitfield & (1 << bit_num)) != 0)?true:false;
}

namespace velma_core_ve_body_types {

class SafeComponent: public RTT::TaskContext {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit SafeComponent(const std::string &name);

    bool configureHook();

    bool startHook();

    void stopHook();

    void updateHook();

    std::string getDiag();

private:
    typedef Eigen::Matrix<double, 7, 1 > ArmJoints;

    const int arm_joints_count_;

    void calculateArmDampingTorque(const ArmJoints &joint_velocity,
        const std::vector<double> &damping_factors, ArmJoints &joint_torque_command);

    void calculateTorsoDampingTorque(double motor_velocity, double &motor_current_command);

    bool isNaN(double d) const;
    bool isInLim(double d, double lo_lim, double hi_lim) const;

    // ROS parameters
	std::vector<double> l_arm_damping_factors_;
	std::vector<double> r_arm_damping_factors_;
	double torso_damping_factor_;

    std::vector<double> arm_q_limits_lo_;
    std::vector<double> arm_q_limits_hi_;
    std::vector<double> arm_dq_limits_;
    std::vector<double> arm_t_limits_;

    // ports
    RTT::OutputPort<ArmJoints > port_rArm_t_out_;
    RTT::OutputPort<ArmJoints > port_lArm_t_out_;
    RTT::OutputPort<double > port_torso_i_out_;

    RTT::OutputPort<double > port_hp_motor_q_out_;
    RTT::OutputPort<double > port_ht_motor_q_out_;

    RTT::OutputPort<std_msgs::Int32 > port_rArm_fri_cmd_out_;
    RTT::OutputPort<std_msgs::Int32 > port_lArm_fri_cmd_out_;

    RTT::InputPort<ArmJoints > port_rArm_dq_in_;
    RTT::InputPort<ArmJoints > port_lArm_dq_in_;
    RTT::InputPort<double > port_torso_dq_in_;
    RTT::InputPort<double > port_hp_motor_q_in_;
    RTT::InputPort<double > port_ht_motor_q_in_;

    RTT::InputPort<lwr_msgs::FriIntfState > port_rArm_fri_in_;
    RTT::InputPort<lwr_msgs::FriRobotState > port_rArm_rob_in_;
    RTT::InputPort<lwr_msgs::FriIntfState > port_lArm_fri_in_;
    RTT::InputPort<lwr_msgs::FriRobotState > port_lArm_rob_in_;

    velma_core_cs_ve_body_msgs::StatusSC sc_out_;
    RTT::OutputPort<velma_core_cs_ve_body_msgs::StatusSC> port_sc_out_;

    ArmJoints rArm_dq_;
    ArmJoints lArm_dq_;
    lwr_msgs::FriIntfState       rArm_fri_state_;
    lwr_msgs::FriRobotState      rArm_robot_state_;
    lwr_msgs::FriIntfState       lArm_fri_state_;
    lwr_msgs::FriRobotState      lArm_robot_state_;

    bool rArm_valid_prev_;
    bool lArm_valid_prev_;

    ros::Time wall_time_prev_;

    uint32_t safe_iterations_;
    bool safe_iterations_over_500_;

    struct timeval time_prev_;
};

SafeComponent::SafeComponent(const std::string &name)
    : TaskContext(name, PreOperational)
    , arm_joints_count_(7)
    , torso_damping_factor_(-1)   // initialize with invalid value, should be later set to >= 0
    , safe_iterations_over_500_(false)
{
    this->ports()->addPort("rArm_t_OUTPORT", port_rArm_t_out_);
    this->ports()->addPort("lArm_t_OUTPORT", port_lArm_t_out_);
    this->ports()->addPort("tMotor_i_OUTPORT", port_torso_i_out_);
    this->ports()->addPort("hpMotor_q_OUTPORT", port_hp_motor_q_out_);
    this->ports()->addPort("htMotor_q_OUTPORT", port_ht_motor_q_out_);
    this->ports()->addPort("rArmFri_OUTPORT", port_rArm_fri_cmd_out_);
    this->ports()->addPort("lArmFri_OUTPORT", port_lArm_fri_cmd_out_);

    this->ports()->addPort("rArm_dq_INPORT", port_rArm_dq_in_);
    this->ports()->addPort("lArm_dq_INPORT", port_lArm_dq_in_);
    this->ports()->addPort("tMotor_dq_INPORT", port_torso_dq_in_);
    this->ports()->addPort("hpMotor_q_INPORT", port_hp_motor_q_in_);
    this->ports()->addPort("htMotor_q_INPORT", port_ht_motor_q_in_);
    this->ports()->addPort("rArmFriIntf_INPORT", port_rArm_fri_in_);
    this->ports()->addPort("rArmFriRobot_INPORT", port_rArm_rob_in_);
    this->ports()->addPort("lArmFriIntf_INPORT", port_lArm_fri_in_);
    this->ports()->addPort("lArmFriRobot_INPORT", port_lArm_rob_in_);

    this->ports()->addPort("sc_OUTPORT", port_sc_out_);

    addProperty("l_arm_damping_factors", l_arm_damping_factors_);
    addProperty("r_arm_damping_factors", r_arm_damping_factors_);
    addProperty("torso_damping_factor", torso_damping_factor_);

    addProperty("arm_q_limits_lo", arm_q_limits_lo_);
    addProperty("arm_q_limits_hi", arm_q_limits_hi_);
    addProperty("arm_dq_limits", arm_dq_limits_);
    addProperty("arm_t_limits", arm_t_limits_);

    addAttribute("safeIterationsOver500", safe_iterations_over_500_);
}

bool SafeComponent::configureHook() {
    Logger::In in("SafeComponent::configureHook");

    if (l_arm_damping_factors_.size() != arm_joints_count_) {
        Logger::log() << Logger::Error <<
            "parameter l_arm_damping_factors is set to illegal value (wrong vector size: " <<
            l_arm_damping_factors_.size() << ")" << Logger::endl;
        return false;
    }

    if (r_arm_damping_factors_.size() != arm_joints_count_) {
        Logger::log() << Logger::Error <<
            "parameter r_arm_damping_factors is set to illegal value (wrong vector size: " <<
            r_arm_damping_factors_.size() << ")" << Logger::endl;
        return false;
    }

    for (int i = 0; i < arm_joints_count_; ++i) {
        if (l_arm_damping_factors_[i] < 0) {
            Logger::log() << Logger::Error <<
                "parameter l_arm_damping_factors[" << i << "] is set to illegal value: " <<
                l_arm_damping_factors_[i] << Logger::endl;
            return false;
        }
        if (r_arm_damping_factors_[i] < 0) {
            Logger::log() << Logger::Error <<
                "parameter r_arm_damping_factors[" << i << "] is set to illegal value: " <<
                r_arm_damping_factors_[i] << Logger::endl;
            return false;
        }
    }

    if (torso_damping_factor_ < 0) {
        Logger::log() << Logger::Error <<
            "parameter torso_damping_factor is set to illegal value: " <<
            torso_damping_factor_ << Logger::endl;
        return false;
    }

    if (arm_q_limits_lo_.size() != arm_joints_count_) {
        Logger::log() << Logger::Error <<
            "parameter arm_q_limits_lo is set to illegal value (wrong vector size: " <<
            arm_q_limits_lo_.size() << ")" << Logger::endl;
        return false;
    }

    if (arm_q_limits_hi_.size() != arm_joints_count_) {
        Logger::log() << Logger::Error <<
            "parameter arm_q_limits_hi is set to illegal value (wrong vector size: " <<
            arm_q_limits_hi_.size() << ")" << Logger::endl;
        return false;
    }

    if (arm_dq_limits_.size() != arm_joints_count_) {
        Logger::log() << Logger::Error <<
            "parameter arm_dq_limits is set to illegal value (wrong vector size: " <<
            arm_dq_limits_.size() << ")" << Logger::endl;
        return false;
    }

    if (arm_t_limits_.size() != arm_joints_count_) {
        Logger::log() << Logger::Error <<
            "parameter arm_t_limits is set to illegal value (wrong vector size: " <<
            arm_t_limits_.size() << ")" << Logger::endl;
        return false;
    }

    return true;
}

bool SafeComponent::startHook() {
    safe_iterations_ = 0;
    safe_iterations_over_500_ = false;
    rArm_valid_prev_ = false;
    lArm_valid_prev_ = false;
    return true;
}

void SafeComponent::stopHook() {
    safe_iterations_ = 0;
    safe_iterations_over_500_ = false;
}

void SafeComponent::calculateArmDampingTorque(const ArmJoints &joint_velocity,
    const std::vector<double> &damping_factors, SafeComponent::ArmJoints &joint_torque_command)
{
    for (int i = 0; i < arm_joints_count_; ++i) {
        joint_torque_command(i) = -damping_factors[i] * joint_velocity(i);
    }
}

void SafeComponent::calculateTorsoDampingTorque(double motor_velocity, double &motor_current_command)
{
    const double torso_gear = 158.0;
    const double torso_trans_mult = 20000.0 * torso_gear / (M_PI * 2.0);
    const double torso_motor_constant = 0.00105;
    double joint_velocity = motor_velocity / torso_trans_mult;
    double motor_torque_command = -torso_damping_factor_ * joint_velocity;
    motor_current_command = motor_torque_command / torso_gear / torso_motor_constant;
}

void SafeComponent::updateHook() {
    //
    // read HW state
    //
    bool rArm_valid = (port_rArm_dq_in_.read(rArm_dq_) == RTT::NewData);

    bool lArm_valid = (port_lArm_dq_in_.read(lArm_dq_) == RTT::NewData);

    double torso_dq;
    bool torso_valid = (port_torso_dq_in_.read(torso_dq) == RTT::NewData);

    double hp_q;
    bool hp_valid = (port_hp_motor_q_in_.read(hp_q) == RTT::NewData);

    double ht_q;
    bool ht_valid = (port_ht_motor_q_in_.read(ht_q) == RTT::NewData);

    rArm_valid &= (port_rArm_fri_in_.read(rArm_fri_state_) == RTT::NewData);
    rArm_valid &= (port_rArm_rob_in_.read(rArm_robot_state_) == RTT::NewData);
    lArm_valid &= (port_lArm_fri_in_.read(lArm_fri_state_) == RTT::NewData);
    lArm_valid &= (port_lArm_rob_in_.read(lArm_robot_state_) == RTT::NewData);

    //
    // write commands to available HW devices
    //

    // generate safe outputs for all operational devices
    if (rArm_valid || rArm_valid_prev_) {
        ArmJoints t;
        calculateArmDampingTorque(rArm_dq_, r_arm_damping_factors_, t);
        port_rArm_t_out_.write(t);
    }

    if (lArm_valid || lArm_valid_prev_) {
        ArmJoints t;
        calculateArmDampingTorque(lArm_dq_, l_arm_damping_factors_, t);
        port_lArm_t_out_.write(t);
    }

    if (torso_valid) {
        double tMotor_i;
        calculateTorsoDampingTorque(torso_dq, tMotor_i);
        port_torso_i_out_.write(tMotor_i);
    }

    if (hp_valid) {
        port_hp_motor_q_out_.write(hp_q);
    }

    if (ht_valid) {
        port_ht_motor_q_out_.write(ht_q);
    }

    // KUKA LWR fri commands
    if (rArm_valid) {
        bool lwrOk = isLwrOk(rArm_robot_state_, rArm_fri_state_);

        // try to switch LWR mode
        if (lwrOk && rArm_fri_state_.state == lwr_msgs::FriIntfState::FRI_STATE_MON) {
            std_msgs::Int32 cmd;
            cmd.data = 1;
            port_rArm_fri_cmd_out_.write(cmd);
        }
    }

    if (lArm_valid) {
        bool lwrOk = isLwrOk(lArm_robot_state_, lArm_fri_state_);

        // try to switch LWR mode
        if (lwrOk && lArm_fri_state_.state == lwr_msgs::FriIntfState::FRI_STATE_MON) {
            std_msgs::Int32 cmd;
            cmd.data = 1;
            port_lArm_fri_cmd_out_.write(cmd);
        }
    }

    //
    // write diagnostics information for other subsystems
    //
    sc_out_.safe_behavior = true;

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

    rArm_valid_prev_ = rArm_valid;
    lArm_valid_prev_ = lArm_valid;
}

}   // velma_core_ve_body_types

ORO_LIST_COMPONENT_TYPE(velma_core_ve_body_types::SafeComponent)

