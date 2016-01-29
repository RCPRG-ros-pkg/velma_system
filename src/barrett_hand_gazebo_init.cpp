#include "barrett_hand_gazebo.h"

    BarrettHandGazebo::BarrettHandGazebo(std::string const& name) : 
        TaskContext(name)
    {

        nh_ = new ros::NodeHandle();
        addProperty("prefix", prefix_);

        // Add required gazebo interfaces
        this->provides("gazebo")->addOperation("configure",&BarrettHandGazebo::gazeboConfigureHook,this,RTT::ClientThread);
        this->provides("gazebo")->addOperation("update",&BarrettHandGazebo::gazeboUpdateHook,this,RTT::ClientThread);

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

        rh_move_hand_ = false;
    }

    BarrettHandGazebo::~BarrettHandGazebo() {
    }

ORO_LIST_COMPONENT_TYPE(BarrettHandGazebo)
ORO_CREATE_COMPONENT_LIBRARY();

