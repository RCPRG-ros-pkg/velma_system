#ifndef BARRETT_HAND_GAZEBO_H__
#define BARRETT_HAND_GAZEBO_H__

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


class BarrettHandGazebo : public RTT::TaskContext
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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

    // public methods
    BarrettHandGazebo(std::string const& name);
    ~BarrettHandGazebo();
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

    double clip(double n, double lower, double upper);

    std::string prefix_;

    ros::NodeHandle *nh_;

    gazebo::physics::ModelPtr model_;
    gazebo::physics::DARTModelPtr model_dart_;
    dart::dynamics::Skeleton *dart_sk_;
    dart::simulation::World *dart_world_;

    // BarrettHand
    std::vector<gazebo::physics::JointPtr>  rh_joints_;

    std::vector<dart::dynamics::Joint*>  rh_joints_dart_;

    bool rh_move_hand_;

    bool rh_clutch_break_[3];

    gazebo::physics::JointController *jc_;

    //! Synchronization
    RTT::os::MutexRecursive gazebo_mutex_;
};

#endif  // BARRETT_HAND_GAZEBO_H__

