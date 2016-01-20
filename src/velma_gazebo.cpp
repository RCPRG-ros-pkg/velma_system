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

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>

#include <kuka_lwr_fri/friComm.h>

typedef Eigen::Matrix<double, 7, 7> Matrix77d;


class VelmaGazebo : public RTT::TaskContext
{
protected:
    ros::NodeHandle *nh_;

    gazebo::physics::ModelPtr model_;
    gazebo::physics::DARTModelPtr model_dart_;
    dart::dynamics::Skeleton *dart_sk_;
    dart::simulation::World *dart_world_;

    std::vector<gazebo::physics::JointPtr>  r_joints_;
    std::vector<gazebo::physics::JointPtr>  l_joints_;
    std::vector<gazebo::physics::JointPtr>  t_joints_;
    std::vector<int> r_indices_;
    std::vector<int> l_indices_;

    std::vector<gazebo::physics::JointPtr>  rh_joints_;
    std::vector<gazebo::physics::JointPtr>  lh_joints_;

    std::vector<dart::dynamics::Joint*>  rh_joints_dart_;
    std::vector<dart::dynamics::Joint*>  lh_joints_dart_;

    bool r_command_mode_;
    bool l_command_mode_;

    gazebo::physics::JointController *jc_;

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

    bool rh_move_hand_;
    bool lh_move_hand_;

    int counts_;
    int counts2_;

    VelmaGazebo(std::string const& name) : 
        TaskContext(name)
    {

        nh_ = new ros::NodeHandle();
        std::cout << "VelmaGazebo ROS node namespace: " << nh_->getNamespace() << std::endl;

        std::cout << "VelmaGazebo ROS node name: " << ros::this_node::getName() << std::endl;
        std::cout << "VelmaGazebo ROS node namespace2: " << ros::this_node::getNamespace() << std::endl;

//        addProperty("LWRlDiag/prefix", prop_prefix);
//        addProperty("parameter", prop_parameter);

        // Add required gazebo interfaces
        this->provides("gazebo")->addOperation("configure",&VelmaGazebo::gazeboConfigureHook,this,RTT::ClientThread);
        this->provides("gazebo")->addOperation("update",&VelmaGazebo::gazeboUpdateHook,this,RTT::ClientThread);
//        this->provides("gazebo")->addOperation("exit",&VelmaGazebo::gazeboExit,this,RTT::ClientThread);

        // right KUKA FRI ports
        this->ports()->addPort("r_JointTorqueCommand",        port_r_JointTorqueCommand_in_).doc("");
        this->ports()->addPort("r_KRL_CMD",                   port_r_KRL_CMD_in_).doc("");
        this->ports()->addPort("r_CartesianWrench",           port_r_CartesianWrench_out_).doc("");
        this->ports()->addPort("r_RobotState",                port_r_RobotState_out_).doc("");
        this->ports()->addPort("r_FRIState",                  port_r_FRIState_out_).doc("");
        this->ports()->addPort("r_JointVelocity",             port_r_JointVelocity_out_).doc("");
        this->ports()->addPort("r_MassMatrix",                port_r_MassMatrix_out_).doc("");
        this->ports()->addPort("r_JointTorque",               port_r_JointTorque_out_).doc("");
        this->ports()->addPort("r_GravityTorque",             port_r_GravityTorque_out_);
        this->ports()->addPort("r_JointPosition",             port_r_JointPosition_out_).doc("");
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
        this->ports()->addPort("l_JointTorqueCommand",        port_l_JointTorqueCommand_in_).doc("");
        this->ports()->addPort("l_KRL_CMD",                   port_l_KRL_CMD_in_).doc("");
        this->ports()->addPort("l_CartesianWrench",           port_l_CartesianWrench_out_).doc("");
        this->ports()->addPort("l_RobotState",                port_l_RobotState_out_).doc("");
        this->ports()->addPort("l_FRIState",                  port_l_FRIState_out_).doc("");
        this->ports()->addPort("l_JointVelocity",             port_l_JointVelocity_out_).doc("");
        this->ports()->addPort("l_MassMatrix",                port_l_MassMatrix_out_).doc("");
        this->ports()->addPort("l_JointTorque",               port_l_JointTorque_out_).doc("");
        this->ports()->addPort("l_GravityTorque",             port_l_GravityTorque_out_);
        this->ports()->addPort("l_JointPosition",             port_l_JointPosition_out_).doc("");
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
        this->ports()->addPort("t_JointTorqueCommand",        port_t_JointTorqueCommand_in_).doc("");
        this->ports()->addPort("t_JointPosition",             port_t_JointPosition_out_).doc("");
        this->ports()->addPort("t_JointVelocity",             port_t_JointVelocity_out_).doc("");
        t_JointTorqueCommand_in_.resize(1);
        t_JointTorqueCommand_in_.setZero();
        t_JointPosition_out_.resize(1);
        t_JointVelocity_out_.resize(1);
        port_t_JointPosition_out_.setDataSample(    t_JointPosition_out_);
        port_t_JointVelocity_out_.setDataSample(    t_JointVelocity_out_);

        // right hand ports
        this->ports()->addPort("rh_q_in",      port_rh_q_in_);
        this->ports()->addPort("rh_v_in",      port_rh_v_in_);
        this->ports()->addPort("rh_t_in",      port_rh_t_in_);
        this->ports()->addPort("rh_mp_in",     port_rh_mp_in_);
        this->ports()->addPort("rh_hold_in",   port_rh_hold_in_);
        this->ports()->addPort("rh_q_out",     port_rh_q_out_);
        this->ports()->addPort("rh_t_out",     port_rh_t_out_);
        this->ports()->addPort("rh_status_out",    port_rh_status_out_);
        //this->ports()->addPort("rh_BHTemp",        port_rh_temp_out_);
        this->ports()->addPort("rh_max_measured_pressure_in", port_rh_max_measured_pressure_in_);
        this->ports()->addPort("rh_reset_fingers", port_rh_reset_in_);
        rh_q_in_.resize(4); rh_q_in_.setZero();
        rh_v_in_.resize(4); rh_v_in_.setZero();
        rh_t_in_.resize(4); rh_t_in_.setZero();
        rh_mp_in_ = 0.0;
        rh_hold_in_ = 0;    // false
        rh_max_measured_pressure_in_.setZero();
        rh_q_out_.resize(8);
        rh_t_out_.resize(8);
        //rh_temp_out_.temp.resize(8);
        port_rh_q_out_.setDataSample(rh_q_out_);
        port_rh_t_out_.setDataSample(rh_t_out_);
        //port_rh_temp_out_.setDataSample(rh_temp_out_);

        // left hand ports
        this->ports()->addPort("lh_q_in",      port_lh_q_in_);
        this->ports()->addPort("lh_v_in",      port_lh_v_in_);
        this->ports()->addPort("lh_t_in",      port_lh_t_in_);
        this->ports()->addPort("lh_mp_in",     port_lh_mp_in_);
        this->ports()->addPort("lh_hold_in",   port_lh_hold_in_);
        this->ports()->addPort("lh_q_out",     port_lh_q_out_);
        this->ports()->addPort("lh_t_out",     port_lh_t_out_);
        this->ports()->addPort("lh_status_out",    port_lh_status_out_);
        //this->ports()->addPort("lh_BHTemp",        port_lh_temp_out_);
        this->ports()->addPort("lh_max_measured_pressure_in", port_lh_max_measured_pressure_in_);
        this->ports()->addPort("lh_reset_fingers", port_lh_reset_in_);
        lh_q_in_.resize(4); lh_q_in_.setZero();
        lh_v_in_.resize(4); lh_v_in_.setZero();
        lh_t_in_.resize(4); lh_t_in_.setZero();
        lh_mp_in_ = 0.0;
        lh_hold_in_ = 0;    // false
        lh_max_measured_pressure_in_.setZero();
        lh_q_out_.resize(8);
        lh_t_out_.resize(8);
        //lh_temp_out_.temp.resize(8);
        port_lh_q_out_.setDataSample(lh_q_out_);
        port_lh_t_out_.setDataSample(lh_t_out_);
        //port_lh_temp_out_.setDataSample(lh_temp_out_);

        rh_move_hand_ = false;
        lh_move_hand_ = false;
        r_command_mode_ = false;
        l_command_mode_ = false;
    }

    ~VelmaGazebo() {
    }

    bool parseDisableCollision(std::string &link1, std::string &link2, TiXmlElement *c)
    {
        const char* link1_str = c->Attribute("link1");
        const char* link2_str = c->Attribute("link2");
        if (link1_str != NULL && link2_str != NULL)
        {
            link1 = link1_str;
            link2 = link2_str;
            return true;
        }
        std::cout<< "disable_collisions has wrong attributes" << std::endl;

        return false;
    }

    bool parseSRDF(const std::string &xml_string, std::vector<std::pair<std::string, std::string> > &disabled_collisions)
    {
        disabled_collisions.clear();

        TiXmlDocument xml_doc;
        xml_doc.Parse(xml_string.c_str());
        if (xml_doc.Error())
        {
//            ROS_ERROR("%s", xml_doc.ErrorDesc());
            xml_doc.ClearError();
            return false;
        }

        TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
        if (!robot_xml)
        {
//            ROS_ERROR("Could not find the 'robot' element in the xml file");
            return false;
        }
        // Get all disable_collisions elements
        for (TiXmlElement* disable_collision_xml = robot_xml->FirstChildElement("disable_collisions"); disable_collision_xml; disable_collision_xml = disable_collision_xml->NextSiblingElement("disable_collisions"))
        {
            std::string link1, link2;
            try {
                if (!parseDisableCollision(link1, link2, disable_collision_xml)) {
                    return false;
                }
                disabled_collisions.push_back(std::make_pair(link1, link2));
            }
            catch (urdf::ParseError &e) {
//                ROS_ERROR("disable_collisions xml is not initialized correctly");
                return false;
            }
        }
        return true;
    }

    void updateHook() {
        // Synchronize with gazeboUpdate()
        RTT::os::MutexLock lock(gazebo_mutex_);
        counts_ = 0;

        counts2_++;
//        std::cout << "c2: " << counts2_ << std::endl;


        port_r_MassMatrix_out_.write(r_MassMatrix_out_);
        port_l_MassMatrix_out_.write(l_MassMatrix_out_);
        port_r_GravityTorque_out_.write(r_GravityTorque_out_);
        port_l_GravityTorque_out_.write(l_GravityTorque_out_);
        port_r_JointTorque_out_.write(r_JointTorque_out_);
        port_l_JointTorque_out_.write(l_JointTorque_out_);
        port_r_JointPosition_out_.write(r_JointPosition_out_);
        port_l_JointPosition_out_.write(l_JointPosition_out_);
        port_t_JointPosition_out_.write(t_JointPosition_out_);
        port_r_JointVelocity_out_.write(r_JointVelocity_out_);
        port_l_JointVelocity_out_.write(l_JointVelocity_out_);
        port_t_JointVelocity_out_.write(t_JointVelocity_out_);

        // FRI comm state
        r_FRIState_out_.quality = FRI_QUALITY_PERFECT;
        l_FRIState_out_.quality = FRI_QUALITY_PERFECT;
        r_FRIState_out_.state = FRI_STATE_CMD;      // FRI_STATE_MON
        l_FRIState_out_.state = FRI_STATE_CMD;
        port_r_FRIState_out_.write(r_FRIState_out_);
        port_l_FRIState_out_.write(l_FRIState_out_);

        // FRI robot state
        r_RobotState_out_.power = 0x7F;
        r_RobotState_out_.error = 0;
        r_RobotState_out_.warning = 0;
        r_RobotState_out_.control == FRI_CTRL_POSITION;     // FRI_CTRL_CART_IMP, FRI_CTRL_JNT_IMP
        l_RobotState_out_.power = 0x7F;
        l_RobotState_out_.error = 0;
        l_RobotState_out_.warning = 0;
        l_RobotState_out_.control == FRI_CTRL_POSITION;     // FRI_CTRL_CART_IMP, FRI_CTRL_JNT_IMP
        port_r_RobotState_out_.write(r_RobotState_out_);
        port_l_RobotState_out_.write(l_RobotState_out_);

        if (port_r_KRL_CMD_in_.read(r_KRL_CMD_in_) == RTT::NewData) {
            if (r_KRL_CMD_in_.data == 1) {
                r_command_mode_ = true;
            }
            else if (r_KRL_CMD_in_.data == 2) {
                r_command_mode_ = false;
            }
        }

        if (port_l_KRL_CMD_in_.read(l_KRL_CMD_in_) == RTT::NewData) {
            if (l_KRL_CMD_in_.data == 1) {
                l_command_mode_ = true;
            }
            else if (l_KRL_CMD_in_.data == 2) {
                l_command_mode_ = false;
            }
        }

        if (port_r_JointTorqueCommand_in_.read(r_JointTorqueCommand_in_) == RTT::NewData) {
//            std::cout << "r: " << r_JointTorqueCommand_in_.transpose() << std::endl;
//            r_JointTorqueCommand_in_.setZero();
        }
        else {
//            std::cout << "r: " << std::endl;
//            r_JointTorqueCommand_in_.setZero();
        }

        if (port_l_JointTorqueCommand_in_.read(l_JointTorqueCommand_in_) == RTT::NewData) {
//            std::cout << "l: " << l_JointTorqueCommand_in_.transpose() << std::endl;
//            l_JointTorqueCommand_in_.setZero();
        }
        else {
//            l_JointTorqueCommand_in_.setZero();
        }

        if (port_t_JointTorqueCommand_in_.read(t_JointTorqueCommand_in_) == RTT::NewData) {
//            std::cout << "t: " << t_JointTorqueCommand_in_.transpose() << std::endl;
//            t_JointTorqueCommand_in_.setZero();
        }
        else {
//            t_JointTorqueCommand_in_.setZero();
        }

        //
        // BarrettHand
        //
        port_rh_q_out_.write(rh_q_out_);
        port_lh_q_out_.write(lh_q_out_);
        port_rh_t_out_.write(rh_t_out_);
        port_lh_t_out_.write(lh_t_out_);

        if (port_rh_q_in_.read(rh_q_in_) == RTT::NewData) {
            std::cout << "rh_q_in_: new data " << rh_q_in_.transpose() << std::endl;
            rh_move_hand_ = true;
        }
        if (port_rh_v_in_.read(rh_v_in_) == RTT::NewData) {
            std::cout << "rh_v_in_: new data " << rh_v_in_.transpose() << std::endl;
        }
        if (port_rh_t_in_.read(rh_t_in_) == RTT::NewData) {
            std::cout << "rh_t_in_: new data " << rh_t_in_.transpose() << std::endl;
        }

        if (port_lh_q_in_.read(lh_q_in_) == RTT::NewData) {
            std::cout << "lh_q_in_: new data " << lh_q_in_.transpose() << std::endl;
            lh_move_hand_ = true;
        }
        port_lh_v_in_.read(lh_v_in_);
        port_lh_t_in_.read(lh_t_in_);
    }

    bool startHook() {
      return true;
    }

    bool configureHook() {
        counts_ = 0;
        counts2_ = 0;
/*
        if (!model_) {
            std::cout << "VelmaGazebo::configureHook: model_ is NULL" << std::endl;
            return false;
        }

        for (int i = 0; i < 7; i++) {
            r_joints_[i]->SetPosition(0, 0.3);
            l_joints_[i]->SetPosition(0, 0.3);
        }

        for (int i = 0; i < 8; i++) {
            rh_joints_[i]->SetPosition(0, 0.3);
            lh_joints_[i]->SetPosition(0, 0.3);
        }
        model_->ResetPhysicsStates();
*/
        std::cout << "VelmaGazebo::configureHook: ok" << std::endl;
        return true;
    }

    bool gazeboConfigureHook(gazebo::physics::ModelPtr model) {

        if(model.get() == NULL) {
            std::cout << "VelmaGazebo::gazeboConfigureHook: the gazebo model is NULL" << std::endl;
            return false;
        }

        model_ = model;

        dart_world_ = boost::dynamic_pointer_cast < gazebo::physics::DARTPhysics > ( gazebo::physics::get_world()->GetPhysicsEngine() ) -> GetDARTWorld();

        model_dart_ = boost::dynamic_pointer_cast < gazebo::physics::DARTModel >(model);
        if (model_dart_.get() == NULL) {
            std::cout << "VelmaGazebo::gazeboConfigureHook: the gazebo model is not a DART model" << std::endl;
            return false;
        }

        dart_sk_ = model_dart_->GetDARTSkeleton();

        std::string hand_joint_names[] = {"_HandFingerOneKnuckleOneJoint", "_HandFingerOneKnuckleTwoJoint", "_HandFingerOneKnuckleThreeJoint",
            "_HandFingerTwoKnuckleOneJoint", "_HandFingerTwoKnuckleTwoJoint", "_HandFingerTwoKnuckleThreeJoint",
            "_HandFingerThreeKnuckleTwoJoint", "_HandFingerThreeKnuckleThreeJoint" };

        jc_ = new gazebo::physics::JointController(model);
        for (int i = 0; i < 8; i++) {
            std::string rh_name( std::string("right") + hand_joint_names[i] );
            std::string lh_name( std::string("left") + hand_joint_names[i] );
            dart_sk_->getJoint(rh_name)->setActuatorType( dart::dynamics::Joint::FORCE );
            dart_sk_->getJoint(lh_name)->setActuatorType( dart::dynamics::Joint::FORCE );
            gazebo::physics::JointPtr rh_joint = model->GetJoint(rh_name);
            if (rh_joint) {
                rh_joints_.push_back(rh_joint);
                rh_joints_dart_.push_back( dart_sk_->getJoint(rh_name) );
            }
            gazebo::common::PID pid0(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            gazebo::common::PID pid(10.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0);//300.0, -300.0);
            jc_->AddJoint(rh_joint);
//            jc_->SetPositionPID(rh_joint->GetScopedName(), pid);
//            jc_->SetVelocityPID(rh_joint->GetScopedName(), pid);

            gazebo::physics::JointPtr lh_joint = model->GetJoint(lh_name);
            if (lh_joint) {
                lh_joints_.push_back(lh_joint);
                lh_joints_dart_.push_back( dart_sk_->getJoint(lh_name) );
            }
            jc_->AddJoint(lh_joint);
//            jc_->SetPositionPID(lh_joint->GetScopedName(), pid);
//            jc_->SetVelocityPID(lh_joint->GetScopedName(), pid);
        }

//        std::cout << rh_joints_[0]->GetScopedName() << std::endl;

//        std::map< std::string,gazebo::common::PID > map = jc_->GetVelocityPIDs();
//        for (std::map< std::string,gazebo::common::PID >::const_iterator it = map.begin(); it != map.end(); it++) {
//            std::cout << "PID map: " << it->first << std::endl;
//        }


        std::string srdf;
        ros::param::get("/robot_semantic_description", srdf);
        dart::collision::CollisionDetector* detector = dart_world_->getConstraintSolver()->getCollisionDetector();

        std::vector<std::pair<std::string, std::string> > disabled_collisions;
        parseSRDF(srdf, disabled_collisions);
        for (std::vector<std::pair<std::string, std::string> >::const_iterator it = disabled_collisions.begin(); it != disabled_collisions.end(); it++) {
            dart::dynamics::BodyNode *bn1 = dart_sk_->getBodyNode(it->first);
            dart::dynamics::BodyNode *bn2 = dart_sk_->getBodyNode(it->second);
            detector->disablePair(bn1, bn2);
        }
        dart_sk_->enableSelfCollision(false);

        // fill in gazebo joints pointer vectors
        for(unsigned int i = 0; i < 7; i++) {
            std::string joint_name = std::string("right_arm_") + std::to_string(i) + "_joint";
            gazebo::physics::JointPtr joint = model->GetJoint(joint_name);
            gazebo::physics::DARTJointPtr joint_dart = boost::dynamic_pointer_cast < gazebo::physics::DARTJoint > ( joint );
            if (joint) {
                this->r_joints_.push_back(joint);
                r_indices_.push_back(joint_dart->GetDARTJoint()->getIndexInSkeleton(0));
            }
            else {
                RTT::log(RTT::Error) << "A joint named " << joint_name.c_str() << " is not part of Mechanism Controlled joints." << RTT::endlog();
                return false;
            }
        }

        for(unsigned int i = 0; i < 7; i++) {
            std::string joint_name = std::string("left_arm_") + std::to_string(i) + "_joint";
            gazebo::physics::JointPtr joint = model->GetJoint(joint_name);
            gazebo::physics::DARTJointPtr joint_dart = boost::dynamic_pointer_cast < gazebo::physics::DARTJoint > ( joint );
            if (joint) {
                this->l_joints_.push_back(joint);
                l_indices_.push_back(joint_dart->GetDARTJoint()->getIndexInSkeleton(0));
            }
            else {
                RTT::log(RTT::Error) << "A joint named " << joint_name.c_str() << " is not part of Mechanism Controlled joints." << RTT::endlog();
                return false;
            }
        }

        for(unsigned int i = 0; i < 1; i++) {
            std::string joint_name = std::string("torso_") + std::to_string(i) + "_joint";
            gazebo::physics::JointPtr joint = model->GetJoint(joint_name);     
            if (joint) {
                this->t_joints_.push_back(joint);
            }
            else {
                RTT::log(RTT::Error) << "A joint named " << joint_name.c_str() << " is not part of Mechanism Controlled joints." << RTT::endlog();
                return false;
            }
        }

        for (int i = 0; i < 7; i++) {
            r_joints_[i]->SetPosition(0, 0.3);
            l_joints_[i]->SetPosition(0, 0.3);
        }

        for (int i = 0; i < 8; i++) {
            rh_joints_[i]->SetPosition(0, 0.3);
            lh_joints_[i]->SetPosition(0, 0.3);
        }
        std::cout << "pos limits: " << rh_joints_dart_[0]->getPositionLowerLimit(0) << "  " << rh_joints_dart_[0]->getPositionUpperLimit(0) << std::endl;
        std::cout << "pos: " << rh_joints_dart_[0]->getPosition(0) << std::endl;

        rh_joints_dart_[0]->setPositionLowerLimit(0, -0.1);
        model_->ResetPhysicsStates();

        std::cout << "pos limits: " << rh_joints_dart_[0]->getPositionLowerLimit(0) << "  " << rh_joints_dart_[0]->getPositionUpperLimit(0) << std::endl;
        std::cout << "pos: " << rh_joints_dart_[0]->getPosition(0) << std::endl;

        return true;
    }


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void gazeboUpdateHook(gazebo::physics::ModelPtr model)
{
    if (!model_dart_) {
        return;
    }

    RTT::os::MutexTryLock trylock(gazebo_mutex_);
    if(!trylock.isSuccessful()) {
        return;
    }

    dart::collision::CollisionDetector* detector = dart_world_->getConstraintSolver()->getCollisionDetector();
    detector->detectCollision(true, true);
    size_t collisionCount = detector->getNumContacts();
    if (collisionCount > 0) {
        std::cout << "col: " << collisionCount << std::endl;
        for(size_t i = 0; i < collisionCount; ++i)
        {
          const dart::collision::Contact& contact = detector->getContact(i);
            std::cout << contact.bodyNode1->getName() << " " << contact.bodyNode2->getName() << std::endl;
//          if(contact.bodyNode1.lock()->getSkeleton() == object
//             || contact.bodyNode2.lock()->getSkeleton() == object)
//          {
//            collision = true;
//            break;
//          }
        }

    }

    counts_++;
//    std::cout << "c1: " << counts_ << std::endl;

    counts2_ = 0;

    // mass matrix
    const Eigen::MatrixXd &mass_matrix = dart_sk_->getMassMatrix();
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            r_MassMatrix_out_(i,j) = mass_matrix(r_indices_[i], r_indices_[j]);
            l_MassMatrix_out_(i,j) = mass_matrix(l_indices_[i], l_indices_[j]);
        }
    }

    // gravity forces
    const Eigen::VectorXd &grav = dart_sk_->getGravityForces();
    for (int i = 0; i < 7; i++) {
        r_GravityTorque_out_(i) = grav[r_indices_[i]];
        l_GravityTorque_out_(i) = grav[l_indices_[i]];
    }

    // external forces
    //const Eigen::VectorXd &ext_f = dart_sk_->getExternalForces();
    for (int i = 0; i < 7; i++) {

        gazebo::physics::JointWrench r_wr, l_wr;
        r_wr = r_joints_[i]->GetForceTorque(0);
        l_wr = l_joints_[i]->GetForceTorque(0);
        if (i%2 == 0) {
            r_JointTorque_out_(i) = r_wr.body2Torque.z;
            l_JointTorque_out_(i) = l_wr.body2Torque.z;
        }
        else {
            r_JointTorque_out_(i) = r_wr.body2Torque.y;
            l_JointTorque_out_(i) = l_wr.body2Torque.y;
        }

//        r_JointTorque_out_(i) = r_joints_[i]->GetForce(0); //ext_f[r_indices_[i]];
//        l_JointTorque_out_(i) = l_joints_[i]->GetForce(0); //ext_f[l_indices_[i]];
    }

    // joint position
    for (int i = 0; i < 7; i++) {
        r_JointPosition_out_(i) = r_joints_[i]->GetAngle(0).Radian();
        l_JointPosition_out_(i) = l_joints_[i]->GetAngle(0).Radian();
    }
    for (int i = 0; i < t_joints_.size(); i++) {
        t_JointPosition_out_(i) = t_joints_[i]->GetAngle(0).Radian();
    }

    // joint velocity
    for (int i = 0; i < 7; i++) {
        r_JointVelocity_out_(i) = r_joints_[i]->GetVelocity(0);
        l_JointVelocity_out_(i) = l_joints_[i]->GetVelocity(0);
    }
    for (int i = 0; i < t_joints_.size(); i++) {
        t_JointVelocity_out_(i) = t_joints_[i]->GetVelocity(0);
    }

    // torque command
    if (r_command_mode_) {
        for (int i = 0; i < 7; i++) {
//            r_joints_[i]->SetForce(0, r_JointTorqueCommand_in_(i));
            r_joints_[i]->SetForce(0, r_JointTorqueCommand_in_(i));
        }
        r_JointTorqueCommand_in_.setZero();
    }

    if (l_command_mode_) {
        for (int i = 0; i < 7; i++) {
            l_joints_[i]->SetForce(0, l_JointTorqueCommand_in_(i));
        }
        l_JointTorqueCommand_in_.setZero();
    }

    for (int i = 0; i < t_joints_.size(); i++) {
        t_joints_[i]->SetForce(0, t_JointTorqueCommand_in_(i));
    }
    t_JointTorqueCommand_in_.setZero();

    //
    // BarrettHand
    //

    const double force_factor = 1000.0;
    // joint position
    for (int i = 0; i < 8; i++) {
        rh_q_out_(i) = rh_joints_[i]->GetAngle(0).Radian();
        lh_q_out_(i) = lh_joints_[i]->GetAngle(0).Radian();
    }

    rh_t_out_[0] = rh_t_out_[3] = rh_joints_[0]->GetForce(0)*force_factor;
    rh_t_out_[1] = rh_t_out_[2] = rh_joints_[1]->GetForce(0)*force_factor;
    rh_t_out_[4] = rh_t_out_[5] = rh_joints_[4]->GetForce(0)*force_factor;
    rh_t_out_[6] = rh_t_out_[7] = rh_joints_[6]->GetForce(0)*force_factor;

    lh_t_out_[0] = lh_t_out_[3] = lh_joints_[0]->GetForce(0)*force_factor;
    lh_t_out_[1] = lh_t_out_[2] = lh_joints_[1]->GetForce(0)*force_factor;
    lh_t_out_[4] = lh_t_out_[5] = lh_joints_[4]->GetForce(0)*force_factor;
    lh_t_out_[6] = lh_t_out_[7] = lh_joints_[6]->GetForce(0)*force_factor;

    // spread joints
    const double vel_trap_angle = 5.0/180.0*3.1415;

    double diff, force;
    int dof_idx, jnt_idx;
    dof_idx = 3;
    jnt_idx = 0;
    diff = rh_q_in_[dof_idx] - rh_joints_[jnt_idx]->GetAngle(0).Radian();
    if (diff > vel_trap_angle) {
        diff = vel_trap_angle;
    }
    if (diff < -vel_trap_angle) {
        diff = -vel_trap_angle;
    }
    force = rh_joints_dart_[jnt_idx]->getForce(0);
//    std::cout << force << std::endl;
/*    if (force > rh_t_in_[dof_idx]/force_factor || force < -rh_t_in_[dof_idx]/force_factor) {
        if (!jc_->SetVelocityTarget(rh_joints_[jnt_idx]->GetScopedName(), 0)) {
            std::cout << "joint " << rh_joints_[jnt_idx]->GetScopedName() << " not found" << std::endl;
        }
        jc_->SetPositionTarget(rh_joints_[jnt_idx]->GetScopedName(), rh_joints_[jnt_idx]->GetAngle(0).Radian());
//        rh_joints_dart_[jnt_idx]->setCommand(0, 0);
    }
    else {
        if (!jc_->SetVelocityTarget(rh_joints_[jnt_idx]->GetScopedName(), rh_v_in_[dof_idx] * diff / vel_trap_angle)) {
            std::cout << "joint " << rh_joints_[jnt_idx]->GetScopedName() << " not found" << std::endl;
        }
        jc_->SetPositionTarget(rh_joints_[jnt_idx]->GetScopedName(), rh_q_in_[dof_idx]);
//        rh_joints_dart_[jnt_idx]->setCommand(0, rh_v_in_[dof_idx] * diff / vel_trap_angle);
    }
*/

    jc_->SetVelocityTarget(rh_joints_[0]->GetScopedName(), 1.0);
//    if (!jc_->SetPositionTarget(rh_joints_[3]->GetScopedName(), -3.5)) {
//        std::cout << "joint " << rh_joints_[3]->GetScopedName() << " not found" << std::endl;
//    }
/*    jc_->SetVelocityTarget(rh_joints_[1]->GetScopedName(), 0);
    jc_->SetPositionTarget(rh_joints_[1]->GetScopedName(), 0);
    jc_->SetVelocityTarget(rh_joints_[2]->GetScopedName(), 0);
    jc_->SetPositionTarget(rh_joints_[2]->GetScopedName(), 0);
    jc_->SetVelocityTarget(rh_joints_[3]->GetScopedName(), 0);
    jc_->SetPositionTarget(rh_joints_[3]->GetScopedName(), 0);
    jc_->SetVelocityTarget(rh_joints_[4]->GetScopedName(), 0);
    jc_->SetPositionTarget(rh_joints_[4]->GetScopedName(), 0);
    jc_->SetVelocityTarget(rh_joints_[5]->GetScopedName(), 0);
    jc_->SetPositionTarget(rh_joints_[5]->GetScopedName(), 0);
    jc_->SetVelocityTarget(rh_joints_[6]->GetScopedName(), 0);
    jc_->SetPositionTarget(rh_joints_[6]->GetScopedName(), 0);
    jc_->SetVelocityTarget(rh_joints_[7]->GetScopedName(), 0);
    jc_->SetPositionTarget(rh_joints_[7]->GetScopedName(), 0);
*/
    jc_->Update();
/*
    double diff, force;
    int dof_idx, jnt_idx;
    dof_idx = 3;
    jnt_idx = 0;
    diff = rh_q_in_[dof_idx] - rh_joints_[jnt_idx]->GetAngle(0).Radian();
    if (diff > vel_trap_angle) {
        diff = vel_trap_angle;
    }
    if (diff < -vel_trap_angle) {
        diff = -vel_trap_angle;
    }
    force = rh_joints_dart_[jnt_idx]->getForce(0);
    if (force > rh_t_in_[dof_idx]/force_factor || force < -rh_t_in_[dof_idx]/force_factor) {
        rh_joints_dart_[jnt_idx]->setCommand(0, 0);
    }
    else {
        rh_joints_dart_[jnt_idx]->setCommand(0, rh_v_in_[dof_idx] * diff / vel_trap_angle);
    }

    dof_idx = 3;
    jnt_idx = 3;
    diff = rh_q_in_[dof_idx] - rh_joints_[jnt_idx]->GetAngle(0).Radian();
    if (diff > vel_trap_angle) {
        diff = vel_trap_angle;
    }
    if (diff < -vel_trap_angle) {
        diff = -vel_trap_angle;
    }
    force = rh_joints_dart_[jnt_idx]->getForce(0);
    if (force > rh_t_in_[dof_idx]/force_factor || force < -rh_t_in_[dof_idx]/force_factor) {
        rh_joints_dart_[jnt_idx]->setCommand(0, 0);
    }
    else {
        rh_joints_dart_[jnt_idx]->setCommand(0, rh_v_in_[dof_idx] * diff / vel_trap_angle);
    }

    // f1 joints
    dof_idx = 0;
    jnt_idx = 1;
    diff = rh_q_in_[dof_idx] - rh_joints_[jnt_idx]->GetAngle(0).Radian();
    if (diff > vel_trap_angle) {
        diff = vel_trap_angle;
    }
    if (diff < -vel_trap_angle) {
        diff = -vel_trap_angle;
    }
    force = rh_joints_dart_[jnt_idx]->getForce(0);
    if (force > rh_t_in_[dof_idx]/force_factor || force < -rh_t_in_[dof_idx]/force_factor) {
        rh_joints_dart_[jnt_idx]->setCommand(0, 0);
    }
    else {
        rh_joints_dart_[jnt_idx]->setCommand(0, rh_v_in_[dof_idx] * diff / vel_trap_angle);
    }

    dof_idx = 0;
    jnt_idx = 2;
    diff = rh_q_in_[dof_idx]*0.3333333 - rh_joints_[jnt_idx]->GetAngle(0).Radian();
    if (diff > vel_trap_angle) {
        diff = vel_trap_angle;
    }
    if (diff < -vel_trap_angle) {
        diff = -vel_trap_angle;
    }
    force = rh_joints_dart_[jnt_idx]->getForce(0);
    if (force > rh_t_in_[dof_idx]/force_factor || force < -rh_t_in_[dof_idx]/force_factor) {
        rh_joints_dart_[jnt_idx]->setCommand(0, 0);
    }
    else {
        rh_joints_dart_[jnt_idx]->setCommand(0, rh_v_in_[dof_idx] * diff / vel_trap_angle);
    }

    // f2 joints
    dof_idx = 1;
    jnt_idx = 4;
    diff = rh_q_in_[dof_idx] - rh_joints_[jnt_idx]->GetAngle(0).Radian();
    if (diff > vel_trap_angle) {
        diff = vel_trap_angle;
    }
    if (diff < -vel_trap_angle) {
        diff = -vel_trap_angle;
    }
    force = rh_joints_dart_[jnt_idx]->getForce(0);
    if (force > rh_t_in_[dof_idx]/force_factor || force < -rh_t_in_[dof_idx]/force_factor) {
        rh_joints_dart_[jnt_idx]->setCommand(0, 0);
    }
    else {
        rh_joints_dart_[jnt_idx]->setCommand(0, rh_v_in_[dof_idx] * diff / vel_trap_angle);
    }

    dof_idx = 1;
    jnt_idx = 5;
    diff = rh_q_in_[dof_idx]*0.3333333 - rh_joints_[jnt_idx]->GetAngle(0).Radian();
    if (diff > vel_trap_angle) {
        diff = vel_trap_angle;
    }
    if (diff < -vel_trap_angle) {
        diff = -vel_trap_angle;
    }
    force = rh_joints_dart_[jnt_idx]->getForce(0);
    if (force > rh_t_in_[dof_idx]/force_factor || force < -rh_t_in_[dof_idx]/force_factor) {
        rh_joints_dart_[jnt_idx]->setCommand(0, 0);
    }
    else {
        rh_joints_dart_[jnt_idx]->setCommand(0, rh_v_in_[dof_idx] * diff / vel_trap_angle);
    }

    // f3 joints
    dof_idx = 2;
    jnt_idx = 6;
    diff = rh_q_in_[dof_idx] - rh_joints_[jnt_idx]->GetAngle(0).Radian();
    if (diff > vel_trap_angle) {
        diff = vel_trap_angle;
    }
    if (diff < -vel_trap_angle) {
        diff = -vel_trap_angle;
    }
    force = rh_joints_dart_[jnt_idx]->getForce(0);
    if (force > rh_t_in_[dof_idx]/force_factor || force < -rh_t_in_[dof_idx]/force_factor) {
        rh_joints_dart_[jnt_idx]->setCommand(0, 0);
    }
    else {
        rh_joints_dart_[jnt_idx]->setCommand(0, rh_v_in_[dof_idx] * diff / vel_trap_angle);
    }

    dof_idx = 2;
    jnt_idx = 7;
    diff = rh_q_in_[dof_idx]*0.3333333 - rh_joints_[jnt_idx]->GetAngle(0).Radian();
    if (diff > vel_trap_angle) {
        diff = vel_trap_angle;
    }
    if (diff < -vel_trap_angle) {
        diff = -vel_trap_angle;
    }
    force = rh_joints_dart_[jnt_idx]->getForce(0);
    if (force > rh_t_in_[dof_idx]/force_factor || force < -rh_t_in_[dof_idx]/force_factor) {
        rh_joints_dart_[jnt_idx]->setCommand(0, 0);
    }
    else {
        rh_joints_dart_[jnt_idx]->setCommand(0, rh_v_in_[dof_idx] * diff / vel_trap_angle);
    }
*/
// TODO:
//    geometry_msgs::Wrench r_CartesianWrench_out_;

}

  private:
    //! Synchronization
    RTT::os::MutexRecursive gazebo_mutex_;

//    std::vector<gazebo::physics::JointPtr>  joints_;
};



ORO_LIST_COMPONENT_TYPE(VelmaGazebo)
ORO_CREATE_COMPONENT_LIBRARY();

