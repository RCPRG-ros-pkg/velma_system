/*
 Copyright (c) 2016, Robot Control and Pattern Recognition Group, Warsaw University of Technology
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

#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/extras/PeriodicActivity.hpp>
#include "rtt/Logger.hpp"
#include <rtt/base/DataObjectLockFree.hpp>

#include "common_behavior/master_service.h"
#include "velma_core_cs_ve_body_msgs/Command.h"
#include "velma_core_ve_body_re_body_msgs/Status.h"
#include "input_data.h"
#include "abstract_behavior.h"
#include "common_predicates.h"

namespace velma_core_ve_body_types {

class VelmaCoreVeBodyMaster : public common_behavior::MasterService {
public:

    void newCmdData(RTT::base::PortInterface*) {
        velma_core_cs_ve_body_msgs::Command data;
        port_cmd_in_.read(data, false);
        // this is synchronized
        cmd_data_.Set(data);
    }

    void newStatusData(RTT::base::PortInterface*) {
        velma_core_ve_body_re_body_msgs::Status data;
        port_status_in_.read(data, false);
        status_data_.Set(data);
        owner_->trigger();
//        std::cout << "newStatusData" << std::endl;
    }

    explicit VelmaCoreVeBodyMaster(RTT::TaskContext* owner)
        : common_behavior::MasterService(owner)
        , port_cmd_in_("velma_core_cs_ve_body_msgs_Command_INPORT")
        , port_cmd_out_("velma_core_cs_ve_body_msgs_Command_OUTPORT")
        , port_status_in_("velma_core_ve_body_re_body_msgs_Status_INPORT")
        , port_status_out_("velma_core_ve_body_re_body_msgs_Status_OUTPORT")
        , cmd_data_( RTT::base::DataObjectBase::Options(2) )      // max two threads
        , owner_(owner)
    {
        cmd_data_.data_sample(velma_core_cs_ve_body_msgs::Command());
        status_data_.data_sample(velma_core_ve_body_re_body_msgs::Status());

        owner->addEventPort(port_cmd_in_, boost::function<void(RTT::base::PortInterface*)>( boost::bind( &VelmaCoreVeBodyMaster::newCmdData, this, _1 ) ) );
//        owner->addPort(port_cmd_in_);
        owner->addPort(port_status_in_);//, boost::function<void(RTT::base::PortInterface*)>( boost::bind( &VelmaCoreVeBodyMaster::newStatusData, this, _1 ) ) );

        owner->addPort(port_cmd_out_);
        owner->addPort(port_status_out_);

        owner->loadService("sim_clock_activity");
        owner->setPeriod(0.001);
    }

    virtual ~VelmaCoreVeBodyMaster() {
    }

//
// OROCOS ports operations
//
    virtual void initBuffers(boost::shared_ptr<common_behavior::InputData >& in_data) const {
        boost::shared_ptr<InputData > in = boost::static_pointer_cast<InputData >(in_data);
        in->cmd_ = velma_core_cs_ve_body_msgs::Command();
        in->status_ = velma_core_ve_body_re_body_msgs::Status();
    }

    virtual bool readStatusPorts(boost::shared_ptr<common_behavior::InputData >& in_data) {
        boost::shared_ptr<InputData > in = boost::static_pointer_cast<InputData >(in_data);

//        // this is synchronized
//        if (status_data_.Get(in->status_) == RTT::NewData) {
//            return true;
//        }
//        in->status_ = velma_core_ve_body_re_body_msgs::Status();
//        return false;

        if (port_status_in_.read(in->status_, false) == RTT::NewData) {
            return true;
        }
        in->status_ = velma_core_ve_body_re_body_msgs::Status();
        return false;
    }

    virtual void writeStatusPorts(boost::shared_ptr<common_behavior::InputData>& in_data) {
        boost::shared_ptr<InputData> in = boost::static_pointer_cast<InputData >(in_data);
        port_status_out_.write(in->status_);
    }

    virtual bool readCommandPorts(boost::shared_ptr<common_behavior::InputData >& in_data) {
        boost::shared_ptr<InputData > in = boost::static_pointer_cast<InputData >(in_data);
        // this is synchronized
        if (cmd_data_.Get(in->cmd_) == RTT::NewData) {
            return true;
        }
        in->cmd_ = velma_core_cs_ve_body_msgs::Command();
        return false;
    }

    virtual void writeCommandPorts(boost::shared_ptr<common_behavior::InputData>& in_data) {
        boost::shared_ptr<InputData> in = boost::static_pointer_cast<InputData >(in_data);
        port_cmd_out_.write(in->cmd_);
    }

    virtual boost::shared_ptr<common_behavior::InputData > getDataSample() const {
        boost::shared_ptr<InputData > ptr(new InputData());
        ptr->cmd_ = velma_core_cs_ve_body_msgs::Command();
        ptr->status_ = velma_core_ve_body_re_body_msgs::Status();
        return boost::static_pointer_cast<common_behavior::InputData >( ptr );
    }

//
// subsystem buffers
//
/*
    // determines if shm ipc interface should be created
    bool enable_ipc_;

    // determines if the buffer component is triggered by new data
    bool event_port_;

    // the prefix used to generate interface classes with macro
    // ORO_LIST_INTERFACE_COMPONENTS
    std::string interface_prefix_;
*/
    virtual void getLowerInputBuffers(std::vector<common_behavior::InputBufferInfo >& info) const {
        info = std::vector<common_behavior::InputBufferInfo >();
        info.push_back(common_behavior::InputBufferInfo(false, false, "velma_core_ve_body_re_body_msgs_Status"));
    }

    virtual void getUpperInputBuffers(std::vector<common_behavior::InputBufferInfo >& info) const {
        info = std::vector<common_behavior::InputBufferInfo >();
        info.push_back(common_behavior::InputBufferInfo(true, true, "velma_core_cs_ve_body_msgs_Command"));
    }

/*
    // determines if shm ipc interface should be created
    bool enable_ipc_;

    // the prefix used to generate interface classes with macro
    // ORO_LIST_INTERFACE_COMPONENTS
    std::string interface_prefix_;
*/
    virtual void getLowerOutputBuffers(std::vector<common_behavior::OutputBufferInfo >& info) const {
        info = std::vector<common_behavior::OutputBufferInfo >();
        info.push_back(common_behavior::OutputBufferInfo(false, "velma_core_ve_body_re_body_msgs_Command"));
    }

    virtual void getUpperOutputBuffers(std::vector<common_behavior::OutputBufferInfo >& info) const {
        info = std::vector<common_behavior::OutputBufferInfo >();
        info.push_back(common_behavior::OutputBufferInfo(true, "velma_core_cs_ve_body_msgs_Status"));
    }

    //
    // FSM parameters
    //
    virtual std::vector<std::string > getStates() const {
        return std::vector<std::string >({"state_velma_core_ve_body_idle", "state_velma_core_ve_body_safe"});
    }

    virtual std::string getInitialState() const {
        return "state_velma_core_ve_body_safe";
    }

    virtual std::vector<std::pair<std::string, std::string > > getLatchedConnections() const {
        return std::vector<std::pair<std::string, std::string > > ({std::make_pair(std::string("velma_core_ve_body_re_body_msgs_StatusConcate"), std::string("safe"))});
//        return std::vector<std::pair<std::string, std::string > > ();
    }

    virtual int getInputDataWaitCycles() const {
        return 1000;
    }

    //
    // error condition info
    //
    // this method is not RT-safe
    virtual std::string getErrorReasonStr(common_behavior::AbstractConditionCauseConstPtr error_reason) const {
        ErrorCauseConstPtr r = boost::dynamic_pointer_cast<const ErrorCause >(error_reason);
/*        std::string result;
        result += (r->getBit(R_LWR_bit)?"R_LWR ":"");
        result += (r->getBit(L_LWR_bit)?"L_LWR ":"");
        result += (r->getBit(R_LWR_CMD_bit)?"R_LWR_CMD ":"");
        result += (r->getBit(L_LWR_CMD_bit)?"L_LWR_CMD ":"");
        result += (r->getBit(STATUS_bit)?"STATUS ":"");
        result += (r->getBit(STATUS_R_LWR_INVALID_bit)?"ST_R_LWR_INV":"");
        result += (r->getBit(STATUS_L_LWR_INVALID_bit)?"ST_L_LWR_INV":"");
        result += (r->getBit(STATUS_R_FT_INVALID_bit)?"ST_R_FT_INV":"");
        result += (r->getBit(STATUS_L_FT_INVALID_bit)?"ST_L_FT_INV":"");
        result += (r->getBit(STATUS_T_MOTOR_INVALID_bit)?"ST_T_MOTOR_INV":"");
        result += (r->getBit(STATUS_HP_MOTOR_INVALID_bit)?"ST_HP_MOTOR_INV":"");
        result += (r->getBit(STATUS_HT_MOTOR_INVALID_bit)?"ST_HT_MOTOR_INV":"");
        result += (r->getBit(LWR_NAN_LIM_bit)?"ST_LWR_NAN_LIM":"");
        result += (r->getBit(COMMAND_bit)?"CMD ":"");
        result += (r->getBit(CMD_T_MOTOR_INVALID_bit)?"CMD_T_MOTOR_INV ":"");
        result += (r->getBit(CMD_HP_MOTOR_INVALID_bit)?"CMD_HP_MOTOR_INV ":"");
        result += (r->getBit(CMD_HT_MOTOR_INVALID_bit)?"CMD_HT_MOTOR_INV ":"");
        result += (r->getBit(CMD_L_ARM_INVALID_bit)?"CMD_L_ARM_INV ":"");
        result += (r->getBit(CMD_R_ARM_INVALID_bit)?"CMD_R_ARM_INV ":"");
        result += (r->getBit(CMD_R_ARM_NAN_bit)?"CMD_R_ARM_NAN ":"");
        result += (r->getBit(CMD_L_ARM_NAN_bit)?"CMD_L_ARM_NAN ":"");
        result += (r->getBit(CMD_R_ARM_LIM_bit)?"CMD_R_ARM_LIM ":"");
        result += (r->getBit(CMD_L_ARM_LIM_bit)?"CMD_L_ARM_LIM ":"");
        result += (r->getBit(CMD_T_MOTOR_T_NAN_bit)?"CMD_T_MOTOR_T_NAN ":"");

        return result;
*/
        return ::getErrorReasonStr(r);
    }

    // this method is not RT-safe
    virtual common_behavior::AbstractConditionCausePtr getErrorReasonSample() const {
        ErrorCausePtr ptr(new ErrorCause());
        return boost::dynamic_pointer_cast<common_behavior::AbstractConditionCause >( ptr );
    }

private:
    RTT::InputPort<velma_core_cs_ve_body_msgs::Command > port_cmd_in_;
    RTT::OutputPort<velma_core_cs_ve_body_msgs::Command > port_cmd_out_;
    RTT::base::DataObjectLockFree<velma_core_cs_ve_body_msgs::Command > cmd_data_;

    RTT::InputPort<velma_core_ve_body_re_body_msgs::Status > port_status_in_;
    RTT::base::DataObjectLockFree<velma_core_ve_body_re_body_msgs::Status > status_data_;
    RTT::OutputPort<velma_core_ve_body_re_body_msgs::Status > port_status_out_;

    RTT::TaskContext* owner_;
};

};  // namespace velma_core_ve_body_types

ORO_SERVICE_NAMED_PLUGIN(velma_core_ve_body_types::VelmaCoreVeBodyMaster, "velma_core_ve_body_master");

