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

#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

#include "Eigen/Dense"

#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>

#include <rtt/Logger.hpp>

#include <barrett_hand_hw_sim/barrett_hand_hw_can.h>

using namespace RTT;

class BarrettHandSim : public RTT::TaskContext
{
protected:
    typedef Eigen::Matrix<double, 4, 1> Dofs;
    typedef Eigen::Matrix<double, 8, 1> Joints;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double          mp_in_;
    int32_t         hold_in_;
    Dofs max_measured_pressure_in_;
    std_msgs::Empty reset_in_;
    uint32_t        status_out_;
    Joints q_out_;
    Joints t_out_;
    //barrett_hand_msgs::BHTemp temp_out_;

    // public methods
    BarrettHandSim(std::string const& name);
    ~BarrettHandSim();
    void updateHook();
    bool startHook();
    bool configureHook();

  protected:

    enum {STATUS_OVERCURRENT1 = 0x0001, STATUS_OVERCURRENT2 = 0x0002, STATUS_OVERCURRENT3 = 0x0004, STATUS_OVERCURRENT4 = 0x0008,
        STATUS_OVERPRESSURE1 = 0x0010, STATUS_OVERPRESSURE2 = 0x0020, STATUS_OVERPRESSURE3 = 0x0040,
        STATUS_TORQUESWITCH1 = 0x0100, STATUS_TORQUESWITCH2 = 0x0200, STATUS_TORQUESWITCH3 = 0x0400,
        STATUS_IDLE1 = 0x1000, STATUS_IDLE2 = 0x2000, STATUS_IDLE3 = 0x4000, STATUS_IDLE4 = 0x8000 };

    double clip(double n, double lower, double upper) const;
    double getFingerAngle(int fidx) const;

    // parameters
    std::string prefix_;
    int can_id_base_;

    bool data_valid_;

    // BarrettHand
    std::vector<std::string> joint_scoped_names_;

//    std::vector<dart::dynamics::Joint*>  joints_dart_;

    std::vector<int > too_big_force_counter_;
    bool move_hand_;

    bool clutch_break_[3];
    double clutch_break_angle_[3];

    double spread_int_;
    double finger_int_[3];

    bool disable_component_;

    BarrettHandHwCAN hw_can_;
};

    BarrettHandSim::BarrettHandSim(std::string const& name)
        : TaskContext(name, RTT::TaskContext::PreOperational)
        , too_big_force_counter_(3, 0)
        , data_valid_(false)
        , disable_component_(false)
        , can_id_base_(-1)
    {
        addProperty("prefix", prefix_);
        addProperty("can_id_base", can_id_base_);
        addProperty("disable_component", disable_component_);

        mp_in_ = 0.0;
        hold_in_ = 0;    // false
        max_measured_pressure_in_.setZero();
        //temp_out_.temp.resize(8);
        status_out_ = STATUS_IDLE1 | STATUS_IDLE2 | STATUS_IDLE3 | STATUS_IDLE4;
        clutch_break_[0] = clutch_break_[1] = clutch_break_[2] = false;
        move_hand_ = false;
    }

    BarrettHandSim::~BarrettHandSim() {
    }

    void BarrettHandSim::updateHook() {

        //
        // BarrettHand
        //
        q_out_.setZero();
        t_out_.setZero();

        hw_can_.jp_[0] = q_out_(1)*50.0*4096.0/2.0/M_PI;
        hw_can_.p_[0] = (q_out_(2) + q_out_(1)) * 4096.0/(1.0/125.0 + 1.0/375.0)/2.0/M_PI;
        hw_can_.jp_[1] = q_out_(4)*4096.0*50.0/2.0/M_PI;
        hw_can_.p_[1] = (q_out_(5) + q_out_(4))*4096.0/(1.0/125.0 + 1.0/375.0)/2.0/M_PI;
        hw_can_.jp_[2] = q_out_(6)*4096.0*50.0/2.0/M_PI;
        hw_can_.p_[2] = (q_out_(7) + q_out_(6))*4096.0/(1.0/125.0 + 1.0/375.0)/2.0/M_PI;
        hw_can_.p_[3] = q_out_(0)*35840.0/M_PI;

        hw_can_.processPuckMsgs();
    }

    bool BarrettHandSim::startHook() {
      return true;
    }

    bool BarrettHandSim::configureHook() {
        Logger::In in("BarrettHandSim::configureHook");
        if (prefix_.empty()) {
            Logger::log() << Logger::Error << "param 'prefix' is empty" << Logger::endl;
            return false;
        }

        if (can_id_base_ < 0) {
            Logger::log() << Logger::Error << "param 'can_id_base' is not set" << Logger::endl;
            return false;
        }

        hw_can_.configure(this, can_id_base_);

        for (int i = 0; i < 3; i++) {
            clutch_break_[i] = false;
        }

        return true;
    }

ORO_LIST_COMPONENT_TYPE(BarrettHandSim)

