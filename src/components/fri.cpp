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

#include "common_interfaces/data_conversion.h"
#include "velma_core_ve_body_re_body_msgs/StatusArmFriIntf.h"
#include "velma_core_ve_body_re_body_msgs/StatusArmFriRobot.h"
#include <kuka_lwr_fri/friComm.h>
#include "rtt/Component.hpp"

void convert(   const tFriIntfState& data_oro,
                velma_core_ve_body_re_body_msgs::StatusArmFriIntf& data_ros) {

    data_ros.timestamp = data_oro.timestamp;
    data_ros.state = data_oro.state;
    data_ros.quality = data_oro.quality;
    data_ros.desiredMsrSampleTime = data_oro.desiredMsrSampleTime;
    data_ros.desiredCmdSampleTime = data_oro.desiredCmdSampleTime;
    data_ros.safetyLimits = data_oro.safetyLimits;
    data_ros.answerRate = data_oro.stat.answerRate;
    data_ros.latency = data_oro.stat.latency;
    data_ros.jitter = data_oro.stat.jitter;
    data_ros.missRate = data_oro.stat.missRate;
    data_ros.missCounter = data_oro.stat.missCounter;
}

void convert(   const velma_core_ve_body_re_body_msgs::StatusArmFriIntf& data_ros,
                tFriIntfState& data_oro) {

    data_oro.timestamp = data_ros.timestamp;
    data_oro.state = data_ros.state;
    data_oro.quality = data_ros.quality;
    data_oro.desiredMsrSampleTime = data_ros.desiredMsrSampleTime;
    data_oro.desiredCmdSampleTime = data_ros.desiredCmdSampleTime;
    data_oro.safetyLimits = data_ros.safetyLimits;
    data_oro.stat.answerRate = data_ros.answerRate;
    data_oro.stat.latency = data_ros.latency;
    data_oro.stat.jitter = data_ros.jitter;
    data_oro.stat.missRate = data_ros.missRate;
    data_oro.stat.missCounter = data_ros.missCounter;
}

REGISTER_DATA_CONVERSION(velma_core_ve_body_re_body_msgs, Status, rArmFriIntf, (velma_core_ve_body_re_body_msgs::StatusArmFriIntf), (tFriIntfState),
{ ::convert(ros, oro); }, { ::convert(oro, ros); } )

REGISTER_DATA_CONVERSION(velma_core_ve_body_re_body_msgs, Status, lArmFriIntf, (velma_core_ve_body_re_body_msgs::StatusArmFriIntf), (tFriIntfState),
{ ::convert(ros, oro); }, { ::convert(oro, ros); } )




void convert(   const tFriRobotState& data_oro,
                velma_core_ve_body_re_body_msgs::StatusArmFriRobot& data_ros) {

    data_ros.power = data_oro.power;
    data_ros.control = data_oro.control;
    data_ros.error = data_oro.error;
    data_ros.warning = data_oro.warning;
    for (int i = 0; i < data_ros.temperature.size(); ++i) {
        data_ros.temperature[i] = data_oro.temperature[i];
    }
}

void convert(   const velma_core_ve_body_re_body_msgs::StatusArmFriRobot& data_ros,
                tFriRobotState& data_oro) {

    data_oro.power = data_ros.power;
    data_oro.control = data_ros.control;
    data_oro.error = data_ros.error;
    data_oro.warning = data_ros.warning;
    for (int i = 0; i < data_ros.temperature.size(); ++i) {
        data_oro.temperature[i] = data_ros.temperature[i];
    }
}

REGISTER_DATA_CONVERSION(velma_core_ve_body_re_body_msgs, Status, rArmFriRobot, (velma_core_ve_body_re_body_msgs::StatusArmFriRobot), (tFriRobotState),
{ ::convert(ros, oro); }, { ::convert(oro, ros); } )

REGISTER_DATA_CONVERSION(velma_core_ve_body_re_body_msgs, Status, lArmFriRobot, (velma_core_ve_body_re_body_msgs::StatusArmFriRobot), (tFriRobotState),
{ ::convert(ros, oro); }, { ::convert(oro, ros); } )

