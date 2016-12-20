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

#ifndef __VELMA_CORE_CS_ABSTRACT_STATE_H__
#define __VELMA_CORE_CS_ABSTRACT_STATE_H__

#include "common_behavior/abstract_state.h"
#include "input_data.h"

namespace velma_task_cs_ros_interface_types {

class StateBase : public common_behavior::StateBase {
public:

    virtual bool checkInitialCondition(
            const boost::shared_ptr<common_behavior::InputData >& in_data,
            const std::vector<RTT::TaskContext*> &components,
            const std::string& prev_state_name,
            bool in_error) const {
        return checkInitialCondition(boost::static_pointer_cast<InputData >(in_data), components, prev_state_name, in_error);
    }

    virtual bool checkInitialCondition(
            const boost::shared_ptr<InputData >& in_data,
            const std::vector<RTT::TaskContext*> &components,
            const std::string& prev_state_name,
            bool in_error) const = 0;

protected:
    StateBase(const std::string& state_name, const std::string& short_state_name, const std::string& behavior_name) :
        common_behavior::StateBase(state_name, short_state_name, behavior_name)
    { }
};

};  // namespace velma_task_cs_ros_interface_types

#endif  // __VELMA_CORE_CS_ABSTRACT_STATE_H__

