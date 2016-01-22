#ifndef VELMA_GAZEBO_H__
#define VELMA_GAZEBO_H__

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/dart/DARTModel.hh>
#include <gazebo/physics/dart/DARTJoint.hh>
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

//#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
//#include <geometry_msgs/Twist.h>

#include <kuka_lwr_fri/friComm.h>

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
    RTT::InputPort<Eigen::VectorXd >    port_t_JointTorqueCommand_in_;  // TorsoTrqSplit.InputPort
    RTT::OutputPort<Eigen::VectorXd >   port_t_JointPosition_out_;      // TorsoPosAggregate.OutputPort
    RTT::OutputPort<Eigen::VectorXd >   port_t_JointVelocity_out_;      // TorsoVelAggregate.OutputPort

    Eigen::VectorXd t_JointTorqueCommand_in_;
    Eigen::VectorXd t_JointPosition_out_;
    Eigen::VectorXd t_JointVelocity_out_;

    // right hand ports
    RTT::InputPort<Eigen::VectorXd>  port_rh_q_in_;
    RTT::InputPort<Eigen::VectorXd>  port_rh_v_in_;
    RTT::InputPort<Eigen::VectorXd>  port_rh_t_in_;
    RTT::InputPort<double>           port_rh_mp_in_;
    RTT::InputPort<int32_t>          port_rh_hold_in_;
    RTT::InputPort<Eigen::Vector4d > port_rh_max_measured_pressure_in_;
    RTT::InputPort<std_msgs::Empty>  port_rh_reset_in_;
    RTT::OutputPort<uint32_t>        port_rh_status_out_;
    RTT::OutputPort<Eigen::VectorXd> port_rh_q_out_;
    RTT::OutputPort<Eigen::VectorXd> port_rh_t_out_;
    //RTT::OutputPort<barrett_hand_controller_msgs::BHTemp> port_rh_temp_out_;

    Eigen::VectorXd rh_q_in_;
    Eigen::VectorXd rh_v_in_;
    Eigen::VectorXd rh_t_in_;
    double          rh_mp_in_;
    int32_t         rh_hold_in_;
    Eigen::Vector4d rh_max_measured_pressure_in_;
    std_msgs::Empty rh_reset_in_;
    uint32_t        rh_status_out_;
    std_msgs::Int32 rh_filter_in_;
    Eigen::VectorXd rh_q_out_;
    Eigen::VectorXd rh_t_out_;
    //barrett_hand_controller_msgs::BHTemp rh_temp_out_;

    // left hand ports
    RTT::InputPort<Eigen::VectorXd>  port_lh_q_in_;
    RTT::InputPort<Eigen::VectorXd>  port_lh_v_in_;
    RTT::InputPort<Eigen::VectorXd>  port_lh_t_in_;
    RTT::InputPort<double>           port_lh_mp_in_;
    RTT::InputPort<int32_t>          port_lh_hold_in_;
    RTT::InputPort<Eigen::Vector4d > port_lh_max_measured_pressure_in_;
    RTT::InputPort<std_msgs::Empty>  port_lh_reset_in_;
    RTT::OutputPort<uint32_t>        port_lh_status_out_;
    RTT::OutputPort<Eigen::VectorXd> port_lh_q_out_;
    RTT::OutputPort<Eigen::VectorXd> port_lh_t_out_;
    //RTT::OutputPort<barrett_hand_controller_msgs::BHTemp> port_lh_temp_out_;

    Eigen::VectorXd lh_q_in_;
    Eigen::VectorXd lh_v_in_;
    Eigen::VectorXd lh_t_in_;
    double          lh_mp_in_;
    int32_t         lh_hold_in_;
    Eigen::Vector4d lh_max_measured_pressure_in_;
    std_msgs::Empty lh_reset_in_;
    uint32_t        lh_status_out_;
    std_msgs::Int32 lh_filter_in_;
    Eigen::VectorXd lh_q_out_;
    Eigen::VectorXd lh_t_out_;
    //barrett_hand_controller_msgs::BHTemp lh_temp_out_;

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

    enum {STATUS_OVERCURRENT1 = 0x0001, STATUS_OVERCURRENT2 = 0x0002, STATUS_OVERCURRENT3 = 0x0004, STATUS_OVERCURRENT4 = 0x0008,
        STATUS_OVERPRESSURE1 = 0x0010, STATUS_OVERPRESSURE2 = 0x0020, STATUS_OVERPRESSURE3 = 0x0040,
        STATUS_TORQUESWITCH1 = 0x0100, STATUS_TORQUESWITCH2 = 0x0200, STATUS_TORQUESWITCH3 = 0x0400,
        STATUS_IDLE1 = 0x1000, STATUS_IDLE2 = 0x2000, STATUS_IDLE3 = 0x4000, STATUS_IDLE4 = 0x8000 };

    bool parseDisableCollision(std::string &link1, std::string &link2, TiXmlElement *c);
    bool parseSRDF(const std::string &xml_string, std::vector<std::pair<std::string, std::string> > &disabled_collisions);
    double clip(double n, double lower, double upper);

    ros::NodeHandle *nh_;

    gazebo::physics::ModelPtr model_;
    gazebo::physics::DARTModelPtr model_dart_;
    dart::dynamics::Skeleton *dart_sk_;
    dart::simulation::World *dart_world_;

    // torso and KUKA LWRs
    std::vector<gazebo::physics::JointPtr>  r_joints_;
    std::vector<gazebo::physics::JointPtr>  l_joints_;
    std::vector<gazebo::physics::JointPtr>  t_joints_;
    std::vector<int> r_indices_;
    std::vector<int> l_indices_;

    bool r_command_mode_;
    bool l_command_mode_;

    // BarrettHands
    std::vector<gazebo::physics::JointPtr>  rh_joints_;
    std::vector<gazebo::physics::JointPtr>  lh_joints_;

    std::vector<dart::dynamics::Joint*>  rh_joints_dart_;
    std::vector<dart::dynamics::Joint*>  lh_joints_dart_;

    bool rh_move_hand_;
    bool lh_move_hand_;

    bool rh_clutch_break_[3];
    bool lh_clutch_break_[3];

    // head
    gazebo::physics::JointPtr head_pan_joint_;
    gazebo::physics::JointPtr head_tilt_joint_;

    gazebo::physics::JointController *jc_;

    //! Synchronization
    RTT::os::MutexRecursive gazebo_mutex_;
};

#endif  // VELMA_GAZEBO_H__

