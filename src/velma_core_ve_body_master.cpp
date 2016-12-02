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
#include "velma_core_cs_ve_body_msgs/Command.h"
#include "velma_core_ve_body_re_body_msgs/Status.h"
#include "input_data.h"

namespace velma_core_ve_body_types {

class VelmaCoreVeBodyMaster : public common_behavior::MasterService {
public:
    explicit VelmaCoreVeBodyMaster(RTT::TaskContext* owner) :
        common_behavior::MasterService(owner),
        port_cmd_in_("command_INPORT"),
        port_status_in_("status_INPORT")
    {
        owner->addPort(port_cmd_in_);
        owner->addPort(port_status_in_);
    }

    virtual ~VelmaCoreVeBodyMaster() {
    }

    virtual void readPorts(boost::shared_ptr<common_behavior::InputData >& in_data) {
        boost::shared_ptr<InputData > in = boost::static_pointer_cast<InputData >(in_data);
        port_cmd_in_.read(in->cmd_);
        port_status_in_.read(in->status_);
    }

    virtual boost::shared_ptr<common_behavior::InputData > getDataSample() {
        boost::shared_ptr<InputData > ptr(new InputData());
        ptr->cmd_ = velma_core_cs_ve_body_msgs::Command();
        ptr->status_ = velma_core_ve_body_re_body_msgs::Status();
        return boost::static_pointer_cast<common_behavior::InputData >( ptr );
    }

/*
class InputBufferInfo {
public:
    // determines if shm ipc interface should be created
    bool enable_ipc_;

    // ipc channel name
    std::string ipc_channel_name_;

    // determines if the buffer component is triggered by new data
    bool event_port_;

    // determines if the buffer component should trigger its slaves
    // even if there is no new data on channel
    bool always_update_peers_;

    // the prefix used to generate interface classes with macro
    // ORO_LIST_INTERFACE_COMPONENTS
    std::string interface_prefix_;

    // the name of the corresponding port in the master component
    std::string master_component_port_name_;
};
*/
    virtual void getLowerInputBuffers(std::vector<common_behavior::InputBufferInfo >& info) const {
        info = std::vector<common_behavior::InputBufferInfo >();
        info.push_back(common_behavior::InputBufferInfo(false, "", false, false, "VelmaCoreVeBodyReBody", port_status_in_.getName()));
    }

    virtual void getUpperInputBuffers(std::vector<common_behavior::InputBufferInfo >& info) const {
        info = std::vector<common_behavior::InputBufferInfo >();
        info.push_back(common_behavior::InputBufferInfo(true, "VelmaCoreCsVeBodyCommand", false, false, "VelmaCoreCsVeBody", port_cmd_in_.getName()));
    }

/*
class OutputBufferInfo {
public:
    // determines if shm ipc interface should be created
    bool enable_ipc_;

    // ipc channel name
    std::string ipc_channel_name_;

    // the prefix used to generate interface classes with macro
    // ORO_LIST_INTERFACE_COMPONENTS
    std::string interface_prefix_;
};
*/
    virtual void getLowerOutputBuffers(std::vector<common_behavior::OutputBufferInfo >& info) const {
        info = std::vector<common_behavior::OutputBufferInfo >();
        info.push_back(common_behavior::OutputBufferInfo(false, "", "VelmaCoreVeBodyReBody"));
    }

    virtual void getUpperOutputBuffers(std::vector<common_behavior::OutputBufferInfo >& info) const {
        info = std::vector<common_behavior::OutputBufferInfo >();
        info.push_back(common_behavior::OutputBufferInfo(true, "VelmaCoreCsVeBodyStatus", "VelmaCoreCsVeBody"));
    }

private:
    RTT::InputPort<velma_core_cs_ve_body_msgs::Command > port_cmd_in_;
    RTT::InputPort<velma_core_ve_body_re_body_msgs::Status > port_status_in_;
};

};  // namespace velma_core_ve_body_types

ORO_SERVICE_NAMED_PLUGIN(velma_core_ve_body_types::VelmaCoreVeBodyMaster, "VelmaCoreVeBodyMaster");

