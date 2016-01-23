#include "velma_gazebo.h"

    VelmaGazebo::VelmaGazebo(std::string const& name) : 
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
        this->ports()->addPort("r_JointTorqueCommand_INPORT",        port_r_JointTorqueCommand_in_).doc("");
        this->ports()->addPort("r_KRL_CMD_INPORT",                   port_r_KRL_CMD_in_).doc("");
        this->ports()->addPort("r_CartesianWrench_OUTPORT",           port_r_CartesianWrench_out_).doc("");
        this->ports()->addPort("r_RobotState_OUTPORT",                port_r_RobotState_out_).doc("");
        this->ports()->addPort("r_FRIState_OUTPORT",                  port_r_FRIState_out_).doc("");
        this->ports()->addPort("r_JointVelocity_OUTPORT",             port_r_JointVelocity_out_).doc("");
        this->ports()->addPort("r_MassMatrix_OUTPORT",                port_r_MassMatrix_out_).doc("");
        this->ports()->addPort("r_JointTorque_OUTPORT",               port_r_JointTorque_out_).doc("");
        this->ports()->addPort("r_GravityTorque_OUTPORT",             port_r_GravityTorque_out_);
        this->ports()->addPort("r_JointPosition_OUTPORT",             port_r_JointPosition_out_).doc("");
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
        this->ports()->addPort("l_JointTorqueCommand_INPORT",         port_l_JointTorqueCommand_in_).doc("");
        this->ports()->addPort("l_KRL_CMD_INPORT",                    port_l_KRL_CMD_in_).doc("");
        this->ports()->addPort("l_CartesianWrench_OUTPORT",           port_l_CartesianWrench_out_).doc("");
        this->ports()->addPort("l_RobotState_OUTPORT",                port_l_RobotState_out_).doc("");
        this->ports()->addPort("l_FRIState_OUTPORT",                  port_l_FRIState_out_).doc("");
        this->ports()->addPort("l_JointVelocity_OUTPORT",             port_l_JointVelocity_out_).doc("");
        this->ports()->addPort("l_MassMatrix_OUTPORT",                port_l_MassMatrix_out_).doc("");
        this->ports()->addPort("l_JointTorque_OUTPORT",               port_l_JointTorque_out_).doc("");
        this->ports()->addPort("l_GravityTorque_OUTPORT",             port_l_GravityTorque_out_);
        this->ports()->addPort("l_JointPosition_OUTPORT",             port_l_JointPosition_out_).doc("");
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
        this->ports()->addPort("t_JointTorqueCommand_INPORT",         port_t_JointTorqueCommand_in_).doc("");
        this->ports()->addPort("t_JointPosition_OUTPORT",             port_t_JointPosition_out_).doc("");
        this->ports()->addPort("t_JointVelocity_OUTPORT",             port_t_JointVelocity_out_).doc("");
        t_JointTorqueCommand_in_.resize(1);
        t_JointTorqueCommand_in_.setZero();
        t_JointPosition_out_.resize(1);
        t_JointVelocity_out_.resize(1);
        port_t_JointPosition_out_.setDataSample(    t_JointPosition_out_);
        port_t_JointVelocity_out_.setDataSample(    t_JointVelocity_out_);

        // right hand ports
        this->ports()->addPort("rh_q_INPORT",      port_rh_q_in_);
        this->ports()->addPort("rh_v_INPORT",      port_rh_v_in_);
        this->ports()->addPort("rh_t_INPORT",      port_rh_t_in_);
        this->ports()->addPort("rh_mp_INPORT",     port_rh_mp_in_);
        this->ports()->addPort("rh_hold_INPORT",   port_rh_hold_in_);
        this->ports()->addPort("rh_q_OUTPORT",     port_rh_q_out_);
        this->ports()->addPort("rh_t_OUTPORT",     port_rh_t_out_);
        this->ports()->addPort("rh_status_OUTPORT",    port_rh_status_out_);
        //this->ports()->addPort("rh_BHTemp",        port_rh_temp_out_);
        this->ports()->addPort("rh_max_measured_pressure_INPORT", port_rh_max_measured_pressure_in_);
        this->ports()->addPort("rh_reset_fingers_INPORT", port_rh_reset_in_);
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
        this->ports()->addPort("lh_q_INPORT",      port_lh_q_in_);
        this->ports()->addPort("lh_v_INPORT",      port_lh_v_in_);
        this->ports()->addPort("lh_t_INPORT",      port_lh_t_in_);
        this->ports()->addPort("lh_mp_INPORT",     port_lh_mp_in_);
        this->ports()->addPort("lh_hold_INPORT",   port_lh_hold_in_);
        this->ports()->addPort("lh_q_OUTPORT",     port_lh_q_out_);
        this->ports()->addPort("lh_t_OUTPORT",     port_lh_t_out_);
        this->ports()->addPort("lh_status_OUTPORT",    port_lh_status_out_);
        //this->ports()->addPort("lh_BHTemp",        port_lh_temp_out_);
        this->ports()->addPort("lh_max_measured_pressure_INPORT", port_lh_max_measured_pressure_in_);
        this->ports()->addPort("lh_reset_fingers_INPORT", port_lh_reset_in_);
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

        // head ports
        this->ports()->addPort("head_pan_motor_position_command_INPORT",        port_hp_q_in_).doc("");
        this->ports()->addPort("head_pan_motor_velocity_command_INPORT",        port_hp_v_in_).doc("");
        this->ports()->addPort("head_pan_motor_current_command_INPORT",         port_hp_c_in_).doc("");
        this->ports()->addPort("head_pan_motor_position_OUTPORT",               port_hp_q_out_).doc("");
        this->ports()->addPort("head_pan_motor_velocity_OUTPORT",               port_hp_v_out_).doc("");
        hp_q_in_ = hp_v_in_ = hp_c_in_ = hp_q_out_ = hp_v_out_ = 0.0;
        this->ports()->addPort("head_tilt_motor_position_command_INPORT",       port_ht_q_in_).doc("");
        this->ports()->addPort("head_tilt_motor_velocity_command_INPORT",       port_ht_v_in_).doc("");
        this->ports()->addPort("head_tilt_motor_current_command_INPORT",        port_ht_c_in_).doc("");
        this->ports()->addPort("head_tilt_motor_position_OUTPORT",              port_ht_q_out_).doc("");
        this->ports()->addPort("head_tilt_motor_velocity_OUTPORT",              port_ht_v_out_).doc("");
        ht_q_in_ = ht_v_in_ = ht_c_in_ = ht_q_out_ = ht_v_out_ = 0.0;

        rh_move_hand_ = false;
        lh_move_hand_ = false;
        r_command_mode_ = false;
        l_command_mode_ = false;
    }

    VelmaGazebo::~VelmaGazebo() {
    }

ORO_LIST_COMPONENT_TYPE(VelmaGazebo)
ORO_CREATE_COMPONENT_LIBRARY();

