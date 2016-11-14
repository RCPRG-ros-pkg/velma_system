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

#include "ft_sensor_gazebo.h"
#include <rtt/Logger.hpp>

using namespace RTT;

void FtSensorGazebo::WrenchKDLToMsg(const KDL::Wrench &in,
                                    geometry_msgs::Wrench &out) const {
  out.force.x = in[0];
  out.force.y = in[1];
  out.force.z = in[2];
  out.torque.x = in[3];
  out.torque.y = in[4];
  out.torque.z = in[5];
}

bool FtSensorGazebo::gazeboConfigureHook(gazebo::physics::ModelPtr model) {
    Logger::In in("FtSensorGazebo::gazeboConfigureHook");

    if(model.get() == NULL) {
        Logger::log() << Logger::Error << "gazebo model is NULL" << Logger::endl;
        return false;
    }

    model_ = model;
/*
    gazebo::physics::DARTModelPtr model_dart = boost::dynamic_pointer_cast < gazebo::physics::DARTModel >(model);
    if (model_dart.get() == NULL) {
        std::cout << "FtSensorGazebo::gazeboConfigureHook: the gazebo model is not a DART model" << std::endl;
        return false;
    }

    dart_sk_ = model_dart->GetDARTSkeleton();
*/
    return true;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void FtSensorGazebo::gazeboUpdateHook(gazebo::physics::ModelPtr model)
{

    if (joint_.get() == NULL) {
        return;
    }
    gazebo::math::Vector3 force = joint_->GetLinkForce(0);
    gazebo::math::Vector3 torque = joint_->GetLinkTorque(0);
    //gazebo::math::Vector3 force = link_->GetWorldForce();
    //gazebo::math::Vector3 torque = link_->GetWorldTorque();
//    Eigen::Vector6d wr = dart_bn_->getBodyForce();
//    KDL::Wrench wr_W = KDL::Wrench( -KDL::Vector(wr(3), wr(4), wr(5)), -KDL::Vector(wr(0), wr(1), wr(2)) );

//    std::cout << "F/T sensor " << joint_name_ << "  f: " << force.x << " " << force.y << " " << force.z << "  t: " << torque.x << " " << torque.y << " " << torque.z << std::endl;

    KDL::Wrench wr_W = KDL::Wrench( -KDL::Vector(force.x, force.y, force.z), -KDL::Vector(torque.x, torque.y, torque.z) );
    KDL::Wrench wr_S = (T_W_S_.Inverse() * wr_W);

    slow_filtered_wrench_ = KDL::Wrench();
    for (int i = 0; i < slow_buffer_size_; i++) {
        slow_filtered_wrench_ += slow_buffer_[i];
    }
    slow_filtered_wrench_ = slow_filtered_wrench_ / slow_buffer_size_;

//        slow_filtered_wrench_ = slow_filtered_wrench_
//            + wr_S / slow_buffer_size_
//            - slow_buffer_[slow_buffer_index_] / slow_buffer_size_;

    slow_buffer_[slow_buffer_index_] = wr_S;
    if ((++slow_buffer_index_) == slow_buffer_size_) {
        slow_buffer_index_ = 0;
    }


    fast_filtered_wrench_ = KDL::Wrench();
    for (int i = 0; i < fast_buffer_size_; i++) {
        fast_filtered_wrench_ += fast_buffer_[i];
    }
    fast_filtered_wrench_ = fast_filtered_wrench_ / fast_buffer_size_;

//        fast_filtered_wrench_ = fast_filtered_wrench_
//            + wr_S / fast_buffer_size_
//            - fast_buffer_[fast_buffer_index_] / fast_buffer_size_;

    fast_buffer_[fast_buffer_index_] = wr_S;
    if ((++fast_buffer_index_) == fast_buffer_size_) {
        fast_buffer_index_ = 0;
    }

    {
        RTT::os::MutexLock lock(gazebo_mutex_);
        WrenchKDLToMsg(wr_S, raw_wrench_out_);
        WrenchKDLToMsg(slow_filtered_wrench_, slow_filtered_wrench_out_);
        WrenchKDLToMsg(fast_filtered_wrench_, fast_filtered_wrench_out_);
        data_valid_ = true;
    }
}

