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

#include <kdl/frames.hpp>

#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>

#include <geometry_msgs/Wrench.h>

#include <kuka_lwr_fri/friComm.h>

class FtSensorSim : public RTT::TaskContext
{
public:
    RTT::OutputPort<geometry_msgs::Wrench> port_raw_wrench_out_;
    RTT::OutputPort<geometry_msgs::Wrench> port_fast_filtered_wrench_out_;
    RTT::OutputPort<geometry_msgs::Wrench> port_slow_filtered_wrench_out_;

    geometry_msgs::Wrench raw_wrench_out_;
    geometry_msgs::Wrench fast_filtered_wrench_out_;
    geometry_msgs::Wrench slow_filtered_wrench_out_;

    // public methods
    FtSensorSim(std::string const& name);
    ~FtSensorSim();
    void updateHook();
    bool startHook();
    bool configureHook();

  protected:

    void WrenchKDLToMsg(const KDL::Wrench &in, geometry_msgs::Wrench &out) const;

    // ROS parameters
    std::string joint_name_;
    std::vector<double> transform_xyz_;
    std::vector<double> transform_rpy_;

    std::vector<KDL::Wrench> slow_buffer_;
    std::vector<KDL::Wrench> fast_buffer_;

    int slow_buffer_index_;
    int fast_buffer_index_;

    int slow_buffer_size_;
    int fast_buffer_size_;

    KDL::Wrench slow_filtered_wrench_;
    KDL::Wrench fast_filtered_wrench_;

    // properties
    std::vector<double> force_limits_;
    RTT::Property<KDL::Wrench> offset_prop_;

    KDL::Frame T_W_S_;

    bool data_valid_;
};

using namespace RTT;

FtSensorSim::FtSensorSim(std::string const& name)
    : TaskContext(name, RTT::TaskContext::PreOperational)
    , slow_buffer_size_(2)
    , fast_buffer_size_(100)
    , slow_buffer_index_(0)
    , fast_buffer_index_(0)
    , data_valid_(false)
    , port_raw_wrench_out_("rawWrench_OUTPORT", false)
    , port_fast_filtered_wrench_out_("fastFilteredWrench_OUTPORT", false)
    , port_slow_filtered_wrench_out_("slowFilteredWrench_OUTPORT", false)
{
    addProperty("joint_name", joint_name_);
    addProperty("transform_xyz", transform_xyz_);
    addProperty("transform_rpy", transform_rpy_);

    // right KUKA FRI ports
    this->ports()->addPort(port_raw_wrench_out_);
    this->ports()->addPort(port_fast_filtered_wrench_out_);
    this->ports()->addPort(port_slow_filtered_wrench_out_);

    slow_buffer_.resize(slow_buffer_size_);
    fast_buffer_.resize(fast_buffer_size_);
}

FtSensorSim::~FtSensorSim() {
}

void FtSensorSim::updateHook() {
    port_raw_wrench_out_.write(raw_wrench_out_);
    port_slow_filtered_wrench_out_.write(slow_filtered_wrench_out_);
    port_fast_filtered_wrench_out_.write(fast_filtered_wrench_out_);
}

bool FtSensorSim::startHook() {
    return true;
}

bool FtSensorSim::configureHook() {
    Logger::In in("FtSensorSim::configureHook");

    if (transform_xyz_.size() != 3) {
        Logger::log() << Logger::Error << "wrong transform_xyz: vector size is " << transform_xyz_.size() << ", should be 3" << Logger::endl;
        return false;
    }

    if (transform_rpy_.size() != 3) {
        Logger::log() << Logger::Error << "wrong transform_rpy: vector size is " << transform_rpy_.size() << ", should be 3" << Logger::endl;
        return false;
    }

    T_W_S_ = KDL::Frame(KDL::Rotation::RPY(transform_rpy_[0], transform_rpy_[1], transform_rpy_[2]), KDL::Vector(transform_xyz_[0], transform_xyz_[1], transform_xyz_[2]));

    return true;
}

ORO_LIST_COMPONENT_TYPE(FtSensorSim)

