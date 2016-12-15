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

#include "common_behavior/master_service.h"
#include "velma_core_cs_task_cs_msgs/Status.h"
#include "input_data.h"

namespace velma_task_cs_ros_interface_types {

class MasterService : public common_behavior::MasterService {
public:
    explicit MasterService(RTT::TaskContext* owner) :
        common_behavior::MasterService(owner),
        port_status_in_("velma_core_cs_task_cs_msgs_Status_INPORT"),
        port_status_out_("velma_core_cs_task_cs_msgs_Status_OUTPORT")
    {
        owner->addPort(port_status_in_);
        owner->addPort(port_status_out_);
        owner->setPeriod(0.01);
    }

    virtual ~MasterService() {
    }

//
// OROCOS ports operations
//

    virtual void initBuffers(boost::shared_ptr<common_behavior::InputData >& in_data) const {
        boost::shared_ptr<InputData > in = boost::static_pointer_cast<InputData >(in_data);
        in->status_ = velma_core_cs_task_cs_msgs::Status();
    }

    virtual bool readStatusPorts(boost::shared_ptr<common_behavior::InputData >& in_data) {
        boost::shared_ptr<InputData > in = boost::static_pointer_cast<InputData >(in_data);
        if (port_status_in_.read(in->status_) == RTT::NewData) {
            return true;
        }
        in->status_ = velma_core_cs_task_cs_msgs::Status();
        return false;
    }

    virtual void writeStatusPorts(boost::shared_ptr<common_behavior::InputData>& in_data) {
        boost::shared_ptr<InputData> in = boost::static_pointer_cast<InputData >(in_data);
        port_status_out_.write(in->status_);
    }

    virtual bool readCommandPorts(boost::shared_ptr<common_behavior::InputData >& in_data) {
        return true;
    }

    virtual void writeCommandPorts(boost::shared_ptr<common_behavior::InputData>& in_data) {
        // do nothing
    }

    virtual boost::shared_ptr<common_behavior::InputData > getDataSample() const {
        boost::shared_ptr<InputData > ptr(new InputData());
        ptr->status_ = velma_core_cs_task_cs_msgs::Status();
        return boost::static_pointer_cast<common_behavior::InputData >( ptr );
    }

//
// subsystem buffers
//
/*
    // determines if shm ipc interface should be created
    bool enable_ipc_;

    // ipc channel name
    std::string ipc_channel_name_;

    // determines if the buffer component is triggered by new data
    bool event_port_;

    // the prefix used to generate interface classes with macro
    // ORO_LIST_INTERFACE_COMPONENTS
    std::string interface_prefix_;
*/
    virtual void getLowerInputBuffers(std::vector<common_behavior::InputBufferInfo >& info) const {
        info = std::vector<common_behavior::InputBufferInfo >();
        info.push_back(common_behavior::InputBufferInfo(true, "velma_core_cs_task_cs_msgs::Status", true, "velma_core_cs_task_cs_msgs::Status"));
    }

    virtual void getUpperInputBuffers(std::vector<common_behavior::InputBufferInfo >& info) const {
        info = std::vector<common_behavior::InputBufferInfo >();
    }

/*
    // determines if shm ipc interface should be created
    bool enable_ipc_;

    // ipc channel name
    std::string ipc_channel_name_;

    // the prefix used to generate interface classes with macro
    // ORO_LIST_INTERFACE_COMPONENTS
    std::string interface_prefix_;
*/
    virtual void getLowerOutputBuffers(std::vector<common_behavior::OutputBufferInfo >& info) const {
        info = std::vector<common_behavior::OutputBufferInfo >();
        info.push_back(common_behavior::OutputBufferInfo(true, "velma_core_cs_task_cs_msgs::Command", "velma_core_cs_task_cs_msgs::Command"));
    }

    virtual void getUpperOutputBuffers(std::vector<common_behavior::OutputBufferInfo >& info) const {
        info = std::vector<common_behavior::OutputBufferInfo >();
    }

    //
    // FSM parameters
    //
    virtual std::vector<std::string > getStates() const {
        return std::vector<std::string >({"state_velma_task_cs_ros_interface_normal"});
    }

    virtual std::string getInitialState() const {
        return "state_velma_task_cs_ros_interface_normal";
    }

    virtual std::vector<std::pair<std::string, std::string > > getLatchedConnections() const {
        return std::vector<std::pair<std::string, std::string > >();
    }

    virtual int getInputDataWaitCycles() const {
        return 0;
    }

private:
    RTT::InputPort<velma_core_cs_task_cs_msgs::Status > port_status_in_;
    RTT::OutputPort<velma_core_cs_task_cs_msgs::Status > port_status_out_;
};

};  // namespace velma_task_cs_ros_interface_types

ORO_SERVICE_NAMED_PLUGIN(velma_task_cs_ros_interface_types::MasterService, "velma_task_cs_ros_interface_master");

