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

#ifndef TORSO_GAZEBO_H__
#define TORSO_GAZEBO_H__

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "Eigen/Dense"

#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>

#include <geometry_msgs/Wrench.h>

#include <kuka_lwr_fri/friComm.h>

typedef Eigen::Matrix<double, 7, 7> Matrix77d;


class TorsoGazebo : public RTT::TaskContext
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // torso ports
    RTT::InputPort<double >    port_t_MotorCurrentCommand_in_;
    RTT::OutputPort<double >   port_t_MotorPosition_out_;
    RTT::OutputPort<double >   port_t_MotorVelocity_out_;

    double t_MotorCurrentCommand_in_;
    double t_MotorPosition_out_;
    double t_MotorVelocity_out_;

    // head ports
    RTT::InputPort<double>      port_hp_q_in_;
    RTT::InputPort<double>      port_hp_v_in_;
    RTT::InputPort<double>      port_hp_c_in_;
    RTT::OutputPort<double>     port_hp_q_out_;
    RTT::OutputPort<double>     port_hp_v_out_;
    RTT::InputPort<double>      port_ht_q_in_;
    RTT::InputPort<double>      port_ht_v_in_;
    RTT::InputPort<double>      port_ht_c_in_;
    RTT::OutputPort<double>     port_ht_q_out_;
    RTT::OutputPort<double>     port_ht_v_out_;

    double hp_q_in_;
    double hp_v_in_;
    double hp_c_in_;
    double hp_q_out_;
    double hp_v_out_;
    double ht_q_in_;
    double ht_v_in_;
    double ht_c_in_;
    double ht_q_out_;
    double ht_v_out_;

    // public methods
    TorsoGazebo(std::string const& name);
    ~TorsoGazebo();
    void updateHook();
    bool startHook();
    bool configureHook();
    bool gazeboConfigureHook(gazebo::physics::ModelPtr model);
    void gazeboUpdateHook(gazebo::physics::ModelPtr model);

  protected:

    double tmp_t_MotorCurrentCommand_in_;
    double tmp_t_MotorPosition_out_;
    double tmp_t_MotorVelocity_out_;

    double tmp_hp_q_in_;
    double tmp_hp_v_in_;
    double tmp_hp_c_in_;
    double tmp_hp_q_out_;
    double tmp_hp_v_out_;
    double tmp_ht_q_in_;
    double tmp_ht_v_in_;
    double tmp_ht_c_in_;
    double tmp_ht_q_out_;
    double tmp_ht_v_out_;

    void setJointsPID();

    ros::NodeHandle *nh_;

    gazebo::physics::ModelPtr model_;

    // head
    gazebo::physics::JointPtr torso_joint_;
    gazebo::physics::JointPtr head_pan_joint_;
    gazebo::physics::JointPtr head_tilt_joint_;

    std::string head_pan_scoped_name_;
    std::string head_tilt_scoped_name_;

    gazebo::physics::JointController *jc_;

    void getJointPositionAndVelocity(Eigen::VectorXd &q, Eigen::VectorXd &dq);
    void getHeadJointPositionAndVelocity(Eigen::VectorXd &q, Eigen::VectorXd &dq);
    void setForces(const Eigen::VectorXd &t);

    //! Synchronization
    RTT::os::MutexRecursive gazebo_mutex_;

    bool data_valid_;
    Eigen::VectorXd q_, dq_;
    Eigen::VectorXd qh_, dqh_;
};

#endif  // TORSO_GAZEBO_H__

