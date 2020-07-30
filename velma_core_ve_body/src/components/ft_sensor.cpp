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

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Component.hpp>

#include <kdl/frames.hpp>

#include <geometry_msgs/Wrench.h>

using namespace RTT;

class FtSensor : public RTT::TaskContext
{
public:
    RTT::InputPort<int32_t > port_FxGage0_in_;
    RTT::InputPort<int32_t > port_FyGage1_in_;
    RTT::InputPort<int32_t > port_FzGage2_in_;
    RTT::InputPort<int32_t > port_TxGage3_in_;
    RTT::InputPort<int32_t > port_TyGage4_in_;
    RTT::InputPort<int32_t > port_TzGage5_in_;
    RTT::InputPort<uint32_t > port_StatusCode_in_;
    RTT::InputPort<uint32_t > port_SampleCounter_in_;

    RTT::OutputPort<uint32_t > port_Control1_out_;
    RTT::OutputPort<uint32_t > port_Control2_out_;

    RTT::OutputPort<geometry_msgs::Wrench > port_w_out_;
    RTT::OutputPort<geometry_msgs::Wrench > port_fw_out_;
    RTT::OutputPort<geometry_msgs::Wrench > port_ffw_out_;

    // public methods
    explicit FtSensor(std::string const& name);
    virtual ~FtSensor();
    void updateHook();
    bool startHook();
    bool configureHook();

  protected:

    // ROS parameters
    double scaling_factor_;

    geometry_msgs::Wrench fw_;
    geometry_msgs::Wrench ffw_;

    std::vector<geometry_msgs::Wrench > slow_buffer_;
    std::vector<geometry_msgs::Wrench > fast_buffer_;

    int slow_buffer_index_;
    int fast_buffer_index_;

    int slow_buffer_size_;
    int fast_buffer_size_;

    std::vector<double> transform_xyz_;
    std::vector<double> transform_rpy_;

    KDL::Frame T_W_S_;
    KDL::Wrench wr_W, wr_S;    
};

FtSensor::FtSensor(std::string const& name)
    : TaskContext(name, RTT::TaskContext::PreOperational)
    , slow_buffer_size_(100)
    , fast_buffer_size_(2)
    , slow_buffer_index_(0)
    , fast_buffer_index_(0)
    , scaling_factor_(0.0)
{
    addProperty("scaling_factor", scaling_factor_);
    addProperty("transform_xyz", transform_xyz_);
    addProperty("transform_rpy", transform_rpy_);      

    this->ports()->addPort("FxGage0_INPORT", port_FxGage0_in_);
    this->ports()->addPort("FyGage1_INPORT", port_FyGage1_in_);
    this->ports()->addPort("FzGage2_INPORT", port_FzGage2_in_);
    this->ports()->addPort("TxGage3_INPORT", port_TxGage3_in_);
    this->ports()->addPort("TyGage4_INPORT", port_TyGage4_in_);
    this->ports()->addPort("TzGage5_INPORT", port_TzGage5_in_);
    this->ports()->addPort("StatusCode_INPORT", port_StatusCode_in_);
    this->ports()->addPort("SampleCounter_INPORT", port_SampleCounter_in_);

    this->ports()->addPort("Control1_OUTPORT", port_Control1_out_);
    this->ports()->addPort("Control2_OUTPORT", port_Control2_out_);

    this->ports()->addPort("w_OUTPORT", port_w_out_);
    this->ports()->addPort("fw_OUTPORT", port_fw_out_);
    this->ports()->addPort("ffw_OUTPORT", port_ffw_out_);  

    slow_buffer_.resize(slow_buffer_size_);
    fast_buffer_.resize(fast_buffer_size_);
}

FtSensor::~FtSensor() {
}

bool FtSensor::configureHook() {
    Logger::In in("FtSensor::configureHook");

    if (scaling_factor_ == 0.0) {
        Logger::log() << Logger::Error << "Parameter 'scaling_factor' is not set." << Logger::endl;
        return false;
    }

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

bool FtSensor::startHook() {
    return true;
}

void FtSensor::updateHook() {
    //TODO: control
    //uint32_t Control1 = 0, Control2 = 0;
    //port_Control1_out_.write(Control1);
    //port_Control2_out_.write(Control2);

    int32_t FxGage0_in;
    int32_t FyGage1_in;
    int32_t FzGage2_in;
    int32_t TxGage3_in;
    int32_t TyGage4_in;
    int32_t TzGage5_in;
    uint32_t StatusCode_in;
    uint32_t SampleCounter_in;

    port_FxGage0_in_.read(FxGage0_in);
    port_FyGage1_in_.read(FyGage1_in);
    port_FzGage2_in_.read(FzGage2_in);
    port_TxGage3_in_.read(TxGage3_in);
    port_TyGage4_in_.read(TyGage4_in);
    port_TzGage5_in_.read(TzGage5_in);

    //TODO: status
    //StatusCode_in = 0;
    //port_StatusCode_in_.read(StatusCode_in);
    //port_SampleCounter_in_.read(SampleCounter_in);

    wr_S = KDL::Wrench(KDL::Vector(FxGage0_in, FyGage1_in, FzGage2_in)*scaling_factor_, KDL::Vector(TxGage3_in, TyGage4_in, TzGage5_in)*scaling_factor_);
    wr_W = T_W_S_ * wr_S;

    geometry_msgs::Wrench w;
    w.force.x = wr_W.force.x();
    w.force.y = wr_W.force.y();
    w.force.z = wr_W.force.z();
    w.torque.x = wr_W.torque.x();
    w.torque.y = wr_W.torque.y();
    w.torque.z = wr_W.torque.z();

    // slow filtered F/T
    fw_.force.x -= slow_buffer_[slow_buffer_index_].force.x / slow_buffer_size_;
    fw_.force.y -= slow_buffer_[slow_buffer_index_].force.y / slow_buffer_size_;
    fw_.force.z -= slow_buffer_[slow_buffer_index_].force.z / slow_buffer_size_;
    fw_.torque.x -= slow_buffer_[slow_buffer_index_].torque.x / slow_buffer_size_;
    fw_.torque.y -= slow_buffer_[slow_buffer_index_].torque.y / slow_buffer_size_;
    fw_.torque.z -= slow_buffer_[slow_buffer_index_].torque.z / slow_buffer_size_;

    fw_.force.x += w.force.x / slow_buffer_size_;
    fw_.force.y += w.force.y / slow_buffer_size_;
    fw_.force.z += w.force.z / slow_buffer_size_;
    fw_.torque.x += w.torque.x / slow_buffer_size_;
    fw_.torque.y += w.torque.y / slow_buffer_size_;
    fw_.torque.z += w.torque.z / slow_buffer_size_;

    slow_buffer_[slow_buffer_index_] = w;
    slow_buffer_index_ = (slow_buffer_index_ + 1) % slow_buffer_size_;

    // fast filtered F/T
    ffw_.force.x -= fast_buffer_[fast_buffer_index_].force.x / fast_buffer_size_;
    ffw_.force.y -= fast_buffer_[fast_buffer_index_].force.y / fast_buffer_size_;
    ffw_.force.z -= fast_buffer_[fast_buffer_index_].force.z / fast_buffer_size_;
    ffw_.torque.x -= fast_buffer_[fast_buffer_index_].torque.x / fast_buffer_size_;
    ffw_.torque.y -= fast_buffer_[fast_buffer_index_].torque.y / fast_buffer_size_;
    ffw_.torque.z -= fast_buffer_[fast_buffer_index_].torque.z / fast_buffer_size_;

    ffw_.force.x += w.force.x / fast_buffer_size_;
    ffw_.force.y += w.force.y / fast_buffer_size_;
    ffw_.force.z += w.force.z / fast_buffer_size_;
    ffw_.torque.x += w.torque.x / fast_buffer_size_;
    ffw_.torque.y += w.torque.y / fast_buffer_size_;
    ffw_.torque.z += w.torque.z / fast_buffer_size_;

    fast_buffer_[fast_buffer_index_] = w;
    fast_buffer_index_ = (fast_buffer_index_ + 1) % fast_buffer_size_;

    port_w_out_.write(w);
    port_fw_out_.write(fw_);
    port_ffw_out_.write(ffw_);
}

ORO_LIST_COMPONENT_TYPE(FtSensor)

