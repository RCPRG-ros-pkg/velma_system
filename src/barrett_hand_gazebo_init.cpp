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
        this->ports()->addPort("q_INPORT",      port_q_in_);
        this->ports()->addPort("v_INPORT",      port_v_in_);
        this->ports()->addPort("t_INPORT",      port_t_in_);
        this->ports()->addPort("mp_INPORT",     port_mp_in_);
        this->ports()->addPort("hold_INPORT",   port_hold_in_);
        this->ports()->addPort("q_OUTPORT",     port_q_out_);
        this->ports()->addPort("t_OUTPORT",     port_t_out_);
        this->ports()->addPort("status_OUTPORT",    port_status_out_);
        //this->ports()->addPort("BHTemp",        port_temp_out_);
        this->ports()->addPort("max_measured_pressure_INPORT", port_max_measured_pressure_in_);
        this->ports()->addPort("reset_fingers_INPORT", port_reset_in_);
        q_in_.resize(4); q_in_.setZero();
        v_in_.resize(4); v_in_.setZero();
        t_in_.resize(4); t_in_.setZero();
        mp_in_ = 0.0;
        hold_in_ = 0;    // false
        max_measured_pressure_in_.setZero();
        q_out_.resize(8);
        t_out_.resize(8);
        //temp_out_.temp.resize(8);
        port_q_out_.setDataSample(q_out_);
        port_t_out_.setDataSample(t_out_);
        //port_temp_out_.setDataSample(temp_out_);

        move_hand_ = false;
    }

    BarrettHandGazebo::~BarrettHandGazebo() {
    }

ORO_LIST_COMPONENT_TYPE(BarrettHandGazebo)
ORO_CREATE_COMPONENT_LIBRARY();

