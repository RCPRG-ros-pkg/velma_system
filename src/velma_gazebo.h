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

#ifndef VELMA_GAZEBO_H__
#define VELMA_GAZEBO_H__

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
//#include <gazebo/physics/dart/DARTModel.hh>
//#include <gazebo/physics/dart/DARTJoint.hh>
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

#include <gazebo/physics/ode/ODECollision.hh>

//#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
//#include <geometry_msgs/Twist.h>

#include <kuka_lwr_fri/friComm.h>

#include "velma_dyn_model.h"

typedef Eigen::Matrix<double, 7, 7> Matrix77d;


class VelmaGazebo : public RTT::TaskContext
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // right KUKA FRI ports
    RTT::InputPort<Eigen::VectorXd >        port_r_JointTorqueCommand_in_;  // FRIr.JointTorqueCommand
    RTT::InputPort<std_msgs::Int32 >        port_r_KRL_CMD_in_;             // FRIr.KRL_CMD
    RTT::OutputPort<tFriRobotState >        port_r_RobotState_out_;         // FRIr.RobotState
    RTT::OutputPort<tFriIntfState >         port_r_FRIState_out_;           // FRIr.FRIState
    RTT::OutputPort<Eigen::VectorXd >       port_r_JointPosition_out_;      // FRIr.JointPosition
    RTT::OutputPort<Eigen::VectorXd >       port_r_JointVelocity_out_;      // FRIr.JointVelocity
    RTT::OutputPort<geometry_msgs::Wrench > port_r_CartesianWrench_out_;    // FRIr.CartesianWrench
    RTT::OutputPort<Matrix77d >             port_r_MassMatrix_out_;         // FRIr.MassMatrix
    RTT::OutputPort<Eigen::VectorXd >       port_r_JointTorque_out_;        // FRIr.JointTorque
    RTT::OutputPort<Eigen::VectorXd >       port_r_GravityTorque_out_;      // FRIr.GravityTorque

    Eigen::VectorXd       r_JointTorqueCommand_in_;
    std_msgs::Int32       r_KRL_CMD_in_;
    tFriRobotState        r_RobotState_out_;
    tFriIntfState         r_FRIState_out_;
    Eigen::VectorXd       r_JointPosition_out_;
    Eigen::VectorXd       r_JointVelocity_out_;
    geometry_msgs::Wrench r_CartesianWrench_out_;
    Matrix77d             r_MassMatrix_out_;
    Eigen::VectorXd       r_JointTorque_out_;
    Eigen::VectorXd       r_GravityTorque_out_;

    // left KUKA FRI ports

    RTT::InputPort<Eigen::VectorXd >        port_l_JointTorqueCommand_in_;  // FRIl.JointTorqueCommand
    RTT::InputPort<std_msgs::Int32 >        port_l_KRL_CMD_in_;             // FRIl.KRL_CMD
    RTT::OutputPort<tFriRobotState >        port_l_RobotState_out_;         // FRIl.RobotState
    RTT::OutputPort<tFriIntfState >         port_l_FRIState_out_;           // FRIl.FRIState
    RTT::OutputPort<Eigen::VectorXd >       port_l_JointPosition_out_;      // FRIl.JointPosition
    RTT::OutputPort<Eigen::VectorXd >       port_l_JointVelocity_out_;      // FRIl.JointVelocity
    RTT::OutputPort<geometry_msgs::Wrench > port_l_CartesianWrench_out_;    // FRIl.CartesianWrench
    RTT::OutputPort<Matrix77d >             port_l_MassMatrix_out_;         // FRIl.MassMatrix
    RTT::OutputPort<Eigen::VectorXd >       port_l_JointTorque_out_;        // FRIl.JointTorque
    RTT::OutputPort<Eigen::VectorXd >       port_l_GravityTorque_out_;      // FRIl.GravityTorque

    Eigen::VectorXd       l_JointTorqueCommand_in_;
    std_msgs::Int32       l_KRL_CMD_in_;
    tFriRobotState        l_RobotState_out_;
    tFriIntfState         l_FRIState_out_;
    Eigen::VectorXd       l_JointPosition_out_;
    Eigen::VectorXd       l_JointVelocity_out_;
    geometry_msgs::Wrench l_CartesianWrench_out_;
    Matrix77d             l_MassMatrix_out_;
    Eigen::VectorXd       l_JointTorque_out_;
    Eigen::VectorXd       l_GravityTorque_out_;

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
    VelmaGazebo(std::string const& name);
    ~VelmaGazebo();
    void updateHook();
    bool startHook();
    bool configureHook();
    bool gazeboConfigureHook(gazebo::physics::ModelPtr model);
    void gazeboUpdateHook(gazebo::physics::ModelPtr model);

  protected:

    // ROS parameters
    std::vector<std::string> init_joint_names_;
	std::vector<double> init_joint_positions_;

    Eigen::VectorXd       tmp_r_JointTorqueCommand_in_;
    std_msgs::Int32       tmp_r_KRL_CMD_in_;
    tFriRobotState        tmp_r_RobotState_out_;
    tFriIntfState         tmp_r_FRIState_out_;
    Eigen::VectorXd       tmp_r_JointPosition_out_;
    Eigen::VectorXd       tmp_r_JointVelocity_out_;
    geometry_msgs::Wrench tmp_r_CartesianWrench_out_;
    Matrix77d             tmp_r_MassMatrix_out_;
    Eigen::VectorXd       tmp_r_JointTorque_out_;
    Eigen::VectorXd       tmp_r_GravityTorque_out_;

    Eigen::VectorXd       tmp_l_JointTorqueCommand_in_;
    std_msgs::Int32       tmp_l_KRL_CMD_in_;
    tFriRobotState        tmp_l_RobotState_out_;
    tFriIntfState         tmp_l_FRIState_out_;
    Eigen::VectorXd       tmp_l_JointPosition_out_;
    Eigen::VectorXd       tmp_l_JointVelocity_out_;
    geometry_msgs::Wrench tmp_l_CartesianWrench_out_;
    Matrix77d             tmp_l_MassMatrix_out_;
    Eigen::VectorXd       tmp_l_JointTorque_out_;
    Eigen::VectorXd       tmp_l_GravityTorque_out_;

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

    enum {STATUS_OVERCURRENT1 = 0x0001, STATUS_OVERCURRENT2 = 0x0002, STATUS_OVERCURRENT3 = 0x0004, STATUS_OVERCURRENT4 = 0x0008,
        STATUS_OVERPRESSURE1 = 0x0010, STATUS_OVERPRESSURE2 = 0x0020, STATUS_OVERPRESSURE3 = 0x0040,
        STATUS_TORQUESWITCH1 = 0x0100, STATUS_TORQUESWITCH2 = 0x0200, STATUS_TORQUESWITCH3 = 0x0400,
        STATUS_IDLE1 = 0x1000, STATUS_IDLE2 = 0x2000, STATUS_IDLE3 = 0x4000, STATUS_IDLE4 = 0x8000 };

    bool parseDisableCollision(std::string &link1, std::string &link2, TiXmlElement *c);
    bool parseSRDF(const std::string &xml_string, std::vector<std::pair<std::string, std::string> > &disabled_collisions);
    double clip(double n, double lower, double upper) const;
    void setInitialPosition(const std::map<std::string, double> &init_q);
    void setJointsDisabledPID();
    void setJointsEnabledPID();

    ros::NodeHandle *nh_;

    gazebo::physics::ModelPtr model_;
//    gazebo::physics::DARTModelPtr model_dart_;
//    dart::dynamics::Skeleton *dart_sk_;
//    dart::simulation::World *dart_world_;

    // torso and KUKA LWRs
    std::vector<gazebo::physics::JointPtr >  r_joints_;
    std::vector<gazebo::physics::JointPtr >  l_joints_;
    std::vector<gazebo::physics::JointPtr >  t_joints_;
    std::vector<int > r_indices_;
    std::vector<int > l_indices_;
    std::vector<int > t_indices_;

//    std::vector<dart::dynamics::Joint* > r_dart_joints_;
//    std::vector<dart::dynamics::Joint* > l_dart_joints_;
//    std::vector<dart::dynamics::Joint* > t_dart_joints_;

    Eigen::VectorXd r_force_prev_;
    Eigen::VectorXd l_force_prev_;

    bool r_command_mode_;
    bool l_command_mode_;

    double right_tool_mass_;
    KDL::Vector right_tool_com_W_;
//    dart::dynamics::BodyNode *right_tool_bn_dart_;
    double left_tool_mass_;
    KDL::Vector left_tool_com_W_;
//    dart::dynamics::BodyNode *left_tool_bn_dart_;

    // head
    gazebo::physics::JointPtr head_pan_joint_;
    gazebo::physics::JointPtr head_tilt_joint_;

    gazebo::physics::JointController *jc_;

    //! Synchronization
    RTT::os::MutexRecursive gazebo_mutex_;

    boost::shared_ptr< gazebo::physics::ODECollision > ode_col1, ode_col2;

    std::map<dGeomID, std::string> gmap;

    void getMassJointPositions(Eigen::VectorXd &q);
    void getExternalForces(Eigen::VectorXd &q);
    void getJointPositionAndVelocity(Eigen::VectorXd &q, Eigen::VectorXd &dq);
    void getHeadJointPositionAndVelocity(Eigen::VectorXd &q, Eigen::VectorXd &dq);
    void setForces(const Eigen::VectorXd &t);
    void getGravComp(Eigen::VectorXd &t);

    DynModelVelma dyn_model_;
    Eigen::VectorXd mass_q_;

    double l_tool_weight_, l_tool_x_, l_tool_y_, l_tool_z_;
    double r_tool_weight_, r_tool_x_, r_tool_y_, r_tool_z_;
};

#endif  // VELMA_GAZEBO_H__

